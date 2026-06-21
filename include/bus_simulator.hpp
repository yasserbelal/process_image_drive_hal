#pragma once
#include "frame_layout.hpp"
#include <array>
#include <cstring>

namespace neura {

/**
 * BusSimulator
 *
 * Models the physical fieldbus as two fixed-size byte arrays:
 *   output_buf_ controller writes commands here, consumed by the bus.
 *   input_buf_  bus writes device feedback here, read by the controller.
 *
 * exchange() is the simulated "bus tick" that runs once per millisecond.
 * Here it is a perfect loopback: every byte written to the output buffer 
 * immediately appears in the input buffer. 

 *
 * Design notes:
 *   - The buffers are plain arrays, not atomics.  
 *     Access is single-threaded in the base assignment.
 *     The optional multi-threaded extension would add
 *     a mutex or lock-free scheme around exchange() (see README).
 *     The middleware calls this. The application never calls it directly.
 *   - No heap allocation: everything lives on the stack / in-place.
 */
class BusSimulator
{
public:
    BusSimulator() 
    {
        output_buf_.fill(0);
        input_buf_.fill(0);
    }

    // Perform one bus exchange (simulated 1 ms tick).
    void exchange() 
    {
        std::memcpy(input_buf_.data(), output_buf_.data(), FRAME_SIZE);
    }

    // Buffer access (used by ProcessImage only) ────────────────────────────────────────────
    uint8_t* output_buf() { return output_buf_.data(); }
    uint8_t* input_buf()  { return input_buf_.data();  }

private:
    std::array<uint8_t, FRAME_SIZE> output_buf_;
    std::array<uint8_t, FRAME_SIZE> input_buf_;
};

} // namespace neura
