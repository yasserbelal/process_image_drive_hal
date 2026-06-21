/**
 * tests/test_all.cpp
 *
 * Self-contained test suite: no external test framework needed.
 * Each test function returns true on pass, false on fail.
 * Run with: ./build/tests/test_all
 *
 * Test coverage:
 *   1. Bus loopback              — exchange copies output to input byte-for-byte
 *   2. Shadow semantics          — feedback reflects the previous cycle's command
 *   3. Little-endian packing     — multi-byte fields encoded correctly
 *   4. Region isolation          — drive writes never touch safety / I/O bytes
 *   5. Enable bit isolation      — set/clear enable bit, other control bits preserved
 *   6. Command round-trip        — set_command()/get_feedback() over boundary
 *                                  values (positive/negative position and torque,
 *                                  two's-complement edge case, enable flag)
 *   7. Temperature byte          — feedback-only field outside DriveCommand
 */

#include "bus_simulator.hpp"
#include "process_image.hpp"
#include "drive_hal.hpp"

#include <cstdio>
#include <cstring>
#include <cassert>

using namespace neura;

// Minimal test harness ───────────────────────────────────────────────────────────────
static int tests_run  = 0;
static int tests_pass = 0;

#define TEST(name) static bool name()
#define RUN(name) do { \
    ++tests_run; \
    if (name()) { ++tests_pass; std::printf("  PASS  " #name "\n"); } \
    else {                      std::printf("  FAIL  " #name "\n"); } \
} while(0)

#define CHECK(expr) do { if (!(expr)) { \
    std::printf("    assertion failed: %s  (%s:%d)\n", #expr, __FILE__, __LINE__); \
    return false; } } while(0)

// Test 1: Bus loopback ───────────────────────────────────────────────────────────────
TEST(test_bus_loopback) 
{
    BusSimulator bus;
    // Write a pattern to every output byte
    for (std::size_t i = 0; i < FRAME_SIZE; ++i)
    {        
        bus.output_buf()[i] = static_cast<uint8_t>(i + 1);
    }    

    bus.exchange();
    for (std::size_t i = 0; i < FRAME_SIZE; ++i)
    {
        CHECK(bus.input_buf()[i] == static_cast<uint8_t>(i + 1));
    }
    return true;
}

// Test 2: Shadow semantics: command written before cycle, read after ─────────────────
TEST(test_shadow_semantics) 
{
    BusSimulator bus;
    ProcessImage pi(bus);
    DriveHAL     hal(pi);

    // Before any cycle: feedback should be all zero
    DriveFeedback fb0 = hal.get_feedback();
    CHECK(fb0.actual_pos == 0);
    CHECK(!fb0.enabled);

    // Write a command, then cycle
    DriveCommand cmd;
    cmd.enable     = true;
    cmd.target_pos = 1000;
    hal.set_command(cmd);
    pi.cycle();

    // After cycle: loopback means feedback now reflects the command
    DriveFeedback fb1 = hal.get_feedback();
    CHECK(fb1.actual_pos == 1000);
    CHECK(fb1.enabled);
    return true;
}

// Test 3: Little-endian packing of int32 ─────────────────────────────────────────────
TEST(test_little_endian_i32) 
{
    BusSimulator bus;
    ProcessImage pi(bus);

    // Write 0x01020304 at byte 10 (target position offset)
    pi.write_i32(DRIVE_TARGET_POS_OFFSET, static_cast<int32_t>(0x01020304));
    pi.cycle();

    // Check raw bytes in input shadow (after loopback)
    const auto& shadow_in = pi.shadow_in();
    CHECK(shadow_in[10] == 0x04);  // least-significant byte first
    CHECK(shadow_in[11] == 0x03);
    CHECK(shadow_in[12] == 0x02);
    CHECK(shadow_in[13] == 0x01);
    return true;
}

// Test 4: Region isolation: drive writes don't corrupt safety / IO bytes ────────────
TEST(test_region_isolation)
 {
    BusSimulator bus;
    ProcessImage pi(bus);
    DriveHAL     hal(pi);

    // Pre-load safety and I/O regions in the output shadow with mock values
    // (simulates another part of the system owning those bytes)
    for (std::size_t byte_idx = SAFETY_OFFSET; byte_idx < SAFETY_OFFSET + SAFETY_SIZE; ++byte_idx)
    {
        pi.shadow_out()[byte_idx] = 0xAA;
    }
    for (std::size_t byte_idx = IO_OFFSET; byte_idx < IO_OFFSET + IO_SIZE; ++byte_idx)
    {
        pi.shadow_out()[byte_idx] = 0xBB;
    }

    // Issue a drive command. Should only affect bytes 8-16
    DriveCommand cmd;
    cmd.enable     = true;
    cmd.target_pos = 1000;
    hal.set_command(cmd);
    pi.cycle();

    // Safety bytes must be unchanged
    const auto& shadow_in = pi.shadow_in();
    for (std::size_t byte_idx = SAFETY_OFFSET; byte_idx < SAFETY_OFFSET + SAFETY_SIZE; ++byte_idx)
    {
        CHECK(shadow_in[byte_idx] == 0xAA);
    }

    // I/O bytes must be unchanged
    for (std::size_t byte_idx = IO_OFFSET; byte_idx < IO_OFFSET + IO_SIZE; ++byte_idx)
    {
        CHECK(shadow_in[byte_idx] == 0xBB);
    }

    return true;
}

// Test 5: Enable bit set/clear preserves other control word bits ──────────────────────
TEST(test_enable_bit_isolation)
 {
    BusSimulator bus;
    ProcessImage pi(bus);
    DriveHAL     hal(pi);

    // Manually set bits 1-7 of the control word
    pi.write_u16(DRIVE_CTRL_WORD_OFFSET, 0x00FE);  // all bits set except bit 0

    // Now set enable (bit 0). Other bits stay the same
    hal.set_enable(true);
    pi.cycle();
    uint16_t status = pi.read_u16(DRIVE_STATUS_WORD_OFFSET);
    CHECK((status & 0x00FF) == 0x00FF);  // bit 0 now set, rest preserved

    // Clear enable. Other bits stay the same
    hal.set_enable(false);
    pi.cycle();
    status = pi.read_u16(DRIVE_STATUS_WORD_OFFSET);
    CHECK((status & 0x00FF) == 0x00FE);  // bit 0 cleared, rest preserved

    return true;
}

// Test 6: Command round-trip, boundary values, including two's-complement ────────────
// Covers: positive/negative position, positive/negative torque, 
// and a full command with every field set.
TEST(test_command_roundtrip_boundary_values)
 {
    BusSimulator bus;
    ProcessImage pi(bus);
    DriveHAL     hal(pi);

    struct Case { bool enable; int32_t pos; int16_t trq; };
    const Case cases[] = 
    {
        {false,  123456,    0},   // positive position
        {false,       0,  750},   // positive torque (75.0 % rated)
        {true,   -32768, -200},   // negative position + negative torque, enabled
        {true,       -1,    0},   // position == -1 (all-ones two's complement)
    };

    for (const auto& c : cases) 
    {
        hal.set_command(DriveCommand{c.enable, c.pos, c.trq, MODE_POSITION});
        pi.cycle();
        DriveFeedback fb = hal.get_feedback();
        CHECK(fb.enabled       == c.enable);
        CHECK(fb.actual_pos    == c.pos);
        CHECK(fb.actual_torque == c.trq);
        CHECK(fb.mode_display  == MODE_POSITION);
    }
    return true;
}

// Test 7: Temperature byte ──────────────────────────────────────────────────────────
TEST(test_temperature_byte) 
{
    BusSimulator bus;
    ProcessImage pi(bus);
    DriveHAL     hal(pi);

    // Manually inject a temperature value into the output shadow
    // (In real hardware the drive writes this; loopback echoes it back)
    pi.write_u8(DRIVE_TEMP_OFFSET, 70);
    pi.cycle();

    DriveFeedback fb = hal.get_feedback();
    CHECK(fb.temperature_c == 70);
    return true;
}

// Main ───────────────────────────────────────────────────────────────────────────────
int main() 
{
    std::printf("=== NEURA Process Image Test Suite ===\n\n");

    RUN(test_bus_loopback);
    RUN(test_shadow_semantics);
    RUN(test_little_endian_i32);
    RUN(test_region_isolation);
    RUN(test_enable_bit_isolation);
    RUN(test_command_roundtrip_boundary_values);
    RUN(test_temperature_byte);

    std::printf("\n%d / %d tests passed\n", tests_pass, tests_run);
    return (tests_pass == tests_run) ? 0 : 1;
}
