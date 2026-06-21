#include "bus_simulator.hpp"
#include "process_image.hpp"
#include "drive_hal.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

/**
 * main.cpp: demonstration of the process image middleware and drive HAL.
 *
 * Runs a small scripted scenario:
 *   1. Enable the drive and command position 1000 counts.
 *   2. After 3 cycles, change position to -500 counts.
 *   3. After 3 more cycles, disable the drive.
 *
 * Every cycle prints the command sent and the feedback received.
 * Because the bus is a loopback, feedback always mirrors the last command
 * after one cycle latency demonstrating the shadow-buffer semantics.
 */

using namespace neura;
using namespace std::chrono_literals;

static void print_feedback(int cycle_num, const DriveFeedback& fb) 
{
    std::printf("  cycle %2d | fb: enabled = %d  pos = %6d  torque = %5d  mode = %d  temp = %d°C\n",
        cycle_num,
        static_cast<int>(fb.enabled),
        fb.actual_pos,
        static_cast<int>(fb.actual_torque),
        static_cast<int>(fb.mode_display),
        static_cast<int>(fb.temperature_c));
}

static void print_command(const DriveCommand& cmd) 
{
    std::printf("  cmd sent |      enable = %d  pos = %6d  torque = %5d\n",
        static_cast<int>(cmd.enable),
        cmd.target_pos,
        static_cast<int>(cmd.target_torque));
}

int main() 
{
    // Construct the three-layer stack ─────────────────────────────────────────────────────
    BusSimulator  bus;
    ProcessImage  pi(bus);
    DriveHAL      hal(pi);

    std::printf("=== NEURA Process Image Middleware Demo ===\n\n");

    // Scenario ────────────────────────────────────────────────────────────────────────────
    // We run 9 cycles with 1 ms spacing to demonstrate:
    //   - Shadow-buffer latency: a command written in cycle N shows in
    //     feedback in cycle N+1 (because loopback happens during cycle()).
    //   - Non-drive bytes are preserved (safety board bytes 0-7, I/O 24-31
    //     remain zero throughout).

    for (int i = 1; i <= 9; ++i)
    {
        DriveCommand cmd;

        if (i <= 3)
        {
            // Phase 1: enable, move to position 1000
            cmd.enable        = true;
            cmd.target_pos    = 1000;
            cmd.target_torque = 100;   // 10.0 % rated
        } else if (i <= 6)
        {
            // Phase 2: still enabled, move to -500
            cmd.enable        = true;
            cmd.target_pos    = -500;
            cmd.target_torque = 50;    // 5.0 % rated
        } else 
        {
            // Phase 3: disable
            cmd.enable        = false;
            cmd.target_pos    = 0;
            cmd.target_torque = 0;
        }

        std::printf(" cycle %d ───────────────────────────────────────────────────\n", i);
        print_command(cmd);

        // Step 1: write command into the shadow
        hal.set_command(cmd);

        // Step 2-4: flush shadow → bus → loopback → capture input shadow
        pi.cycle();

        // Step 5: read feedback (reflects this cycle's command due to loopback)
        const DriveFeedback fb = hal.get_feedback();
        print_feedback(i, fb);

        // Verify non-drive bytes are not corrupted ─────────────────────────────────────
        const auto& shadow_in = pi.shadow_in();
        
        bool safety_ok = true;
        for (std::size_t byte_idx = SAFETY_OFFSET; byte_idx < SAFETY_OFFSET + SAFETY_SIZE; ++byte_idx) 
        {
            if (shadow_in[byte_idx] != 0) 
            {
                safety_ok = false; 
                break; 
            }
        }

        bool io_ok = true;
        for (std::size_t byte_idx = IO_OFFSET; byte_idx < IO_OFFSET + IO_SIZE; ++byte_idx) 
        {
            if (shadow_in[byte_idx] != 0) 
            { 
                io_ok = false; break; 
            }
        }

        std::printf("  region   | safety_bytes_clean = %d  io_bytes_clean = %d\n",
                    static_cast<int>(safety_ok), static_cast<int>(io_ok));

        std::this_thread::sleep_for(1ms);
    }

    std::printf("\n=== Demo complete ===\n");
    return 0;
}
