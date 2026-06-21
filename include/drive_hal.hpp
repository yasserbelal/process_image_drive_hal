#pragma once
#include "process_image.hpp"
#include "frame_layout.hpp"
#include <cstdint>

namespace neura {

/**
 * DriveCommand
 *
 * Plain struct the application fills in and passes to DriveHAL::set_command().
 * Keeping it separate from DriveHAL means the application never calls multiple
 * setters in a half-written state.
 * The whole command is applied atomically to the shadow in one call.
 */
struct DriveCommand 
{
    bool    enable        = false;
    int32_t target_pos    = 0;              
    int16_t target_torque = 0;              // 0.1 % of rated torque
    uint8_t mode          = MODE_POSITION;
};

/**
 * DriveFeedback
 *
 * Plain struct returned by DriveHAL::get_feedback().
 * Reflects the input shadow after the most recent exchange cycle.
 */
struct DriveFeedback 
{
    bool    enabled       = false;
    int32_t actual_pos    = 0;              
    int16_t actual_torque = 0;              // 0.1 % of rated torque
    uint8_t mode_display  = 0;
    uint8_t temperature_c = 0;              
};

/**
 * DriveHAL  (Hardware Abstraction Layer for the servo drive)
 *
 * The sole public interface for application code that wants to talk to the servo drive.  
 * It knows about frame offsets, endianness, and bit fields, the application does not.
 *
 * Typical application loop
 * ────────────────────────
 *   DriveCommand cmd;
 *   cmd.enable       = true;
 *   cmd.target_pos   = 1000;
 *   cmd.target_torque = 0;
 *
 *   hal.set_command(cmd);                      // writes to shadow_out
 *   pi.cycle();                                // shadow_out → bus → shadow_in
 *   DriveFeedback fb = hal.get_feedback();     // reads from shadow_in
 *
 * Design notes
 * ────────────
 *   - DriveHAL never calls pi.cycle() itself.  
 *     The application (or a dedicated cycle thread) owns the timing.  
 *     This keeps the HAL stateless with respect to time.
 *   - Non-drive bytes 0-7 and 24-63 (safety board, I/O board) are never touched.
 *     set_command() modifies only bytes 8-16.
 *   - All field access goes through ProcessImage's endian-safe helpers.
 */
class DriveHAL 
{
public:
    explicit DriveHAL(ProcessImage& pi) : pi_(pi) {}

    // Command API ──────────────────────────────────────────────────────────────────────────

    /**
     * Write a complete drive command into the output shadow.
     * Takes effect on the next pi.cycle() call.
     * Only drive bytes (8-16) are modified. Other regions are untouched.
     */
    void set_command(const DriveCommand& cmd) 
    {
        // Build control word: set or clear enable bit, preserve other bits.
        uint16_t ctrl = pi_.read_ctrl_word();

        if (cmd.enable) 
        {
            ctrl |=  CTRL_ENABLE_BIT;
        } else 
        {
            ctrl &= ~CTRL_ENABLE_BIT;
        }

        pi_.write_u16(DRIVE_CTRL_WORD_OFFSET,   ctrl);
        pi_.write_i32(DRIVE_TARGET_POS_OFFSET,  cmd.target_pos);
        pi_.write_i16(DRIVE_TARGET_TRQ_OFFSET,  cmd.target_torque);
        pi_.write_u8 (DRIVE_MODE_OFFSET,        cmd.mode);
    }

    /**
     * Set only the enable bit without touching other fields.
     * Useful when one just wants to toggle enable in a safety monitor.
     */
    void set_enable(bool enable)
    {
        uint16_t ctrl = pi_.read_ctrl_word();

        if (enable) 
        { 
            ctrl |=  CTRL_ENABLE_BIT;
        } else 
        {
            ctrl &= ~CTRL_ENABLE_BIT;
        }

        pi_.write_u16(DRIVE_CTRL_WORD_OFFSET, ctrl);
    }

    // Feedback API ─────────────────────────────────────────────────────────────────────────

    /**
     * Read a complete drive feedback snapshot from the input shadow.
     * Reflects the state captured during the most recent pi.cycle() call.
     */
    DriveFeedback get_feedback() const 
    {
        DriveFeedback fb;
        const uint16_t status = pi_.read_u16(DRIVE_STATUS_WORD_OFFSET);
        fb.enabled       = (status & STATUS_ENABLED_BIT) != 0;
        fb.actual_pos    = pi_.read_i32(DRIVE_ACTUAL_POS_OFFSET);
        fb.actual_torque = pi_.read_i16(DRIVE_ACTUAL_TRQ_OFFSET);
        fb.mode_display  = pi_.read_u8(DRIVE_MODE_DISP_OFFSET);
        fb.temperature_c = pi_.read_u8(DRIVE_TEMP_OFFSET);
        return fb;
    }

    /**
     * Read only whether the drive reports itself as enabled.
     */
    bool is_enabled() const 
    {
        return (pi_.read_u16(DRIVE_STATUS_WORD_OFFSET) & STATUS_ENABLED_BIT) != 0;
    }

    /**
     * Read only the actual position. Used by safety monitor if implenented.
     */
    int32_t get_actual_position() const 
    {
        return pi_.read_i32(DRIVE_ACTUAL_POS_OFFSET);
    }

    /**
     * Read only the actual torque.
     */
    int16_t get_actual_torque() const 
    {
        return pi_.read_i16(DRIVE_ACTUAL_TRQ_OFFSET);
    }

private:
    ProcessImage& pi_;
};

} // namespace neura
