# NEURA Robotics: Process Image Middleware & Drive HAL

Take-home assignment solution.  
**Author:** Yasser Belal  
**Language:** C++17. No external dependencies. Header-only library.

---

## What this implements

| Component | File | Description |
|---|---|---|
| Frame layout constants | `include/frame_layout.hpp` | Single source of truth for all byte offsets, sizes, and bit masks |
| Bus simulator | `include/bus_simulator.hpp` | Two fixed-size buffers & 1 ms loopback exchange method |
| Process image middleware | `include/process_image.hpp` | Shadow buffer, endian-safe read/write helpers, cycle logic |
| Drive HAL | `include/drive_hal.hpp` | Public API for the servo drive which hides all offsets and packing |
| Demo application | `src/main.cpp` | 9-cycle scripted scenario with console output |
| Test suite | `tests/test_all.cpp` | 7 self-contained tests, zero external dependencies |

---

## File structure

```
neura_process_image/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/
│   ├── frame_layout.hpp     ← byte offsets and constants
│   ├── bus_simulator.hpp    ← two buffers + loopback exchange
│   ├── process_image.hpp    ← shadow buffer + cycle logic
│   └── drive_hal.hpp        ← public drive API
├── src/
│   └── main.cpp             ← demo application
└── tests/
    └── test_all.cpp         ← 7 unit tests, no external framework
```

## Build and run

### Prerequisites

- CMake ≥ 3.16  
- A C++17 compiler (GCC ≥ 9, Clang ≥ 10, MSVC 2019+)

### Steps

```bash
git clone https://github.com/yasserbelal/process_image_drive_hal.git
cd process_image_drive_hal

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run tests
./build/test_all

# Run demo
./build/demo
```

Expected test output:
```
=== NEURA Process Image Test Suite ===

  PASS  test_bus_loopback
  PASS  test_shadow_semantics
  PASS  test_little_endian_i32
  PASS  test_region_isolation
  PASS  test_enable_bit_isolation
  PASS  test_command_roundtrip_boundary_values
  PASS  test_temperature_byte

7 / 7 tests passed
```

---

## Architecture

```
┌─────────────────────────────────────────────┐
│              Application code               │
│       DriveHAL::set_command(cmd)            │
│       DriveHAL::get_feedback() → fb         │
└──────────────┬──────────────────────────────┘
               │  reads/writes named fields
┌──────────────▼──────────────────────────────┐
│              DriveHAL                       │
│   Knows: byte offsets, bit masks,           │
│          endianness, field widths           │
└──────────────┬──────────────────────────────┘
               │  write_u16 / write_i32 / read_i32 /…
┌──────────────▼──────────────────────────────┐
│              ProcessImage                   │
│   shadow_out[64]  ←-- application writes    │
│   shadow_in[64]   --→ application reads     │
│   cycle(): out → bus → loopback → bus → in  │
└──────────────┬──────────────────────────────┘
               │  output_buf / input_buf pointers
┌──────────────▼──────────────────────────────┐
│              BusSimulator                   │
│   output_buf[64]                            │
│   input_buf[64]                             │
│   exchange(): memcpy(input, output)         │
└─────────────────────────────────────────────┘
```

---

## Design decisions and assumptions

### 1. Single source of truth for offsets (`frame_layout.hpp`)

Every byte offset, bit mask, and field size is defined exactly once in `frame_layout.hpp` as a 
`static constexpr`. Neither `ProcessImage` nor `DriveHAL` contains hardcoded values.

### 2. What the application sees if it reads before the next cycle

If application code calls `set_command()` and then immediately calls `get_feedback()` **without** 
calling `cycle()` in between, it reads `shadow_in` which still holds the **previous** cycle's feedback. 
The new command is sitting in `shadow_out` waiting for the next `cycle()` call to flush it to the bus.

### 3. Endianness

All multi-byte fields are packed and unpacked byte-by-byte using explicit shift-and-mask arithmetic. 

### 4. Non-drive bytes are never touched

`DriveHAL` only calls `write_*` helpers at offsets 8–16. 
Bytes 0–7 (safety board) and bytes 24–63 (I/O, unspecified) are never written by the drive API. 
This is verified in `test_region_isolation`, which pre-fills safety and I/O bytes with mock 
values and asserts they are unchanged after a drive command cycle.

### 5. Read-modify-write on the control word

Optional `set_enable()` reads the current output shadow value of the control word, sets or clears bit 0 only, 
and writes it back. This preserves any other bits in the control word that may have been set 
independently (e.g. a quick-stop bit set by a safety function). 

---

## Optional: multi-threaded extension

> This section describes the design. The base implementation is single-threaded as required.

### The scenario
 
Two threads run concurrently:
 
- **Application thread**: It writes a command via `hal.set_command()` or `hal.set_enable()` and captures feedback. 
- **Safety monitor thread**: runs independently, at its own pace. It periodically calls `hal.get_actual_position()`,
  and if the position leaves a configured range, calls `hal.set_enable(false)` to clear only the enable bit.
  Once the position is back inside range, it allows `hal.set_enable(true)` again.
  
Both threads touch the shared process image: the application thread writes the full command into `shadow_out` and reads `shadow_in`.
The safety monitor thread also writes `shadow_out` (only the enable bit, via `set_enable()`) and reads `shadow_in` (only position,
via `get_actual_position()`).

The application thread's write of the full command and the safety monitor's write of just the enable bit can land
on `shadow_out` at the same time.
That overlap is a data race by the C++ memory model, which is undefined behaviour and can corrupt the
control word in practice (torn reads, reordering).

### What can go wrong
 
Imagine the application thread executes `set_command()`, which does a read-modify-write of the 
control word to set the enable bit alongside writing position and torque:
 
```
read ctrl_word  →  0x0000
set bit 0       →  0x0001
write ctrl_word →  shadow_out[8] = 0x01, shadow_out[9] = 0x00
```
 
If the safety monitor thread simultaneously executes `set_enable(false)`,
which performs its own read-modify-write of the same two bytes:
 
```
read ctrl_word  →  0x0001
clear bit 0     →  0x0000
write ctrl_word →  shadow_out[8] = 0x00, shadow_out[9] = 0x00
```
 
The two read-modify-write sequences can interleave in any order. 
Possible outcomes: the safety monitor's clear is silently lost (application thread's
write happens last, leaving the drive enabled despite an out-of-range position) or 
a torn write produces a control word that was never intended by either thread. 
Either outcome is unacceptable for a safety function losing the disable request is the more dangerous of the two.

### Correct solution: a mutex protecting the output shadow
  
```cpp
// In ProcessImage:
mutable std::mutex shadow_out_mutex_;
 
void write_u16(size_t offset, uint16_t value) {
    std::lock_guard<std::mutex> lk(shadow_out_mutex_);
    // ... byte-by-byte write as before
}
 
void cycle()
{
    // Step 1: flush application commands to the wire output buffer.
    {
        std::lock_guard<std::mutex> lk(shadow_out_mutex_);  // constructor: lock()
        std::memcpy(bus_.output_buf(), shadow_out_.data(), FRAME_SIZE);
    }   // destructor: unlock()

    // Step 2: bus exchange (loopback).
    bus_.exchange();

    // Step 3: capture device feedback into the shadow input buffer.
    // shadow_in is written only by cycle(), which only the application thread calls.
    // So no lock needed.
    std::memcpy(shadow_in_.data(), bus_.input_buf(), FRAME_SIZE);
}
```
 
Both `set_command()` (called by the application thread) and `set_enable()` (called by the safety monitor thread) 
acquire `shadow_out_mutex_` for the duration of their read-modify-write sequence not just for an individual 
byte write. So one thread's full update to the control word always completes before the other's begins. 
This eliminates the interleaving shown above: whichever thread acquires the lock first finishes its complete 
read-modify-write before the other can start, so the safety monitor's disable request can never be silently
overwritten mid-sequence.
 
For `shadow_in` (feedback), only the application thread's `cycle()` call writes it, and the safety monitor 
thread only reads it (via `get_actual_position()`). Because there is exactly one writer, a mutex
can often be omitted.

---

### Safety monitor thread
 
```cpp
void safety_monitor(DriveHAL& hal, int32_t pos_min, int32_t pos_max)
{
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const int32_t pos = hal.get_actual_position();
        if (pos < pos_min || pos > pos_max)
        {
            hal.set_enable(false);   // mutex-protected write to shadow_out
        }
        // When back in range: do nothing here. The application thread's
        // own set_command()/set_enable(true) calls are what re-enable.
    }
}
```
 
`set_enable()` acquires `shadow_out_mutex_`, so it is safe to call from the safety monitor thread 
while the application thread concurrently calls `set_command()` in its own loop. 
If the application thread requests enable in the same cycle the safety monitor requests disable, 
the mutex ensures one read-modify-write fully completes before the other begins.
Last writer wins, which for a single enable bit is the correct, safe behaviour.

```

