#pragma once
#include <cstdint>
#include <cstddef>

/**
 * frame_layout.hpp
 *
 * Frame map (output and input share the same byte positions):
 *   Bytes  0-7  : Safety board   (read-only from application perspective)
 *   Bytes  8-23 : Servo drive    (abstracted by DriveHAL)
 *   Bytes 24-31 : I/O board      (read-only from application perspective)
 *   Bytes 32-63 : Unspecified / reserved
 *
 * All multi-byte integers are little-endian.
 */

namespace neura 
{
    // Frame dimensions ─────────────────────────────────────────────────────────────────────
    static constexpr std::size_t FRAME_SIZE = 64;

    // Region boundaries ────────────────────────────────────────────────────────────────────
    static constexpr std::size_t SAFETY_OFFSET = 0;
    static constexpr std::size_t SAFETY_SIZE   = 8;

    static constexpr std::size_t DRIVE_OFFSET  = 8;
    static constexpr std::size_t DRIVE_SIZE    = 16;             // bytes 8-23

    static constexpr std::size_t IO_OFFSET     = 24;
    static constexpr std::size_t IO_SIZE       = 8;

    // Drive output (command) fields, relative to byte 0 of frame ───────────────────────────
    static constexpr std::size_t DRIVE_CTRL_WORD_OFFSET   = 8;   // uint16_t
    static constexpr std::size_t DRIVE_TARGET_POS_OFFSET  = 10;  // int32_t
    static constexpr std::size_t DRIVE_TARGET_TRQ_OFFSET  = 14;  // int16_t  (0.1 % rated)
    static constexpr std::size_t DRIVE_MODE_OFFSET        = 16;  // uint8_t

    // Drive input (feedback) fields, same positions ────────────────────────────────────────
    static constexpr std::size_t DRIVE_STATUS_WORD_OFFSET = 8;   // uint16_t
    static constexpr std::size_t DRIVE_ACTUAL_POS_OFFSET  = 10;  // int32_t
    static constexpr std::size_t DRIVE_ACTUAL_TRQ_OFFSET  = 14;  // int16_t
    static constexpr std::size_t DRIVE_MODE_DISP_OFFSET   = 16;  // uint8_t
    static constexpr std::size_t DRIVE_TEMP_OFFSET        = 17;  // uint8_t  (°C)

    // Control / status word bit masks ──────────────────────────────────────────────────────
    static constexpr uint16_t CTRL_ENABLE_BIT  = (1u << 0);      // bit 0 = enable request
    static constexpr uint16_t STATUS_ENABLED_BIT = (1u << 0);    // bit 0 = enabled

    // Drive operating modes ────────────────────────────────────────────────────────────────
    static constexpr uint8_t MODE_POSITION = 8;

} // namespace neura
