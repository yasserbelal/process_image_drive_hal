#pragma once
#include "bus_simulator.hpp"
#include "frame_layout.hpp"
#include <array>
#include <cstring>
#include <cstdint>

namespace neura {

/**
 * ProcessImage
 *
 * Owns the shadow buffer and orchestrates one complete exchange cycle.
 *
 * Shadow buffer vs wire buffers
 * ─────────────────────────────
 * The application never touches the bus simulator's wire buffers directly.
 * Instead it reads and writes the shadow, which is a private copy that
 * reflects the most recently exchanged state.
 *
 * One cycle (called by the application's timing loop every 1 ms):
 *   1. shadow_out → wire output   (flush commands to bus)
 *   2. bus.exchange()             (loopback: output → input)
 *   3. wire input → shadow_in     (capture feedback from bus)
 *
 * Consequence for the application:
 *   If one writes a command and immediately reads feedback within the same cycle
 *   (before calling cycle()), one will read the "previous" cycle's feedback,
 *   not the effect of one's new command.  
 *   The command one wrote takes effect on the "next" exchange. 
 *   This is intentional and matches real fieldbus behaviour (see README "Design Decisions").
 *
 * Endianness helpers
 * ──────────────────
 * All helpers use explicit byte-by-byte packing so they work correctly
 * regardless of host endianness. The frame spec says little-endian, so
 * byte 0 of a multi-byte field is the least-significant byte.
 */
class ProcessImage 
{
public:
    explicit ProcessImage(BusSimulator& bus) : bus_(bus) 
    {
        shadow_out_.fill(0);
        shadow_in_.fill(0);
    }

    /**
     * Run one complete exchange cycle.
     * Call this exactly once per 1 ms tick from the application loop.
     */
    void cycle()
    {
        // Step 1: flush application commands to the wire output buffer.
        std::memcpy(bus_.output_buf(), shadow_out_.data(), FRAME_SIZE);

        // Step 2: bus exchange (loopback).
        bus_.exchange();

        // Step 3: capture device feedback into the shadow input buffer.
        std::memcpy(shadow_in_.data(), bus_.input_buf(), FRAME_SIZE);
    }

    // Write helpers (application → shadow_out) ─────────────────────────────────────────────

    /**
     * Write a 16-bit unsigned value at byte offset in the output shadow.
     * Little-endian: low byte first.
     */
    void write_u16(std::size_t offset, uint16_t value) 
    {
        shadow_out_[offset]     = static_cast<uint8_t>(value & 0xFF);
        shadow_out_[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    /**
     * Write a 32-bit signed value at byte offset in the output shadow.
     * Little-endian: low byte first.
     */
    void write_i32(std::size_t offset, int32_t value) 
    {
        const auto uval = static_cast<uint32_t>(value);
        shadow_out_[offset]     = static_cast<uint8_t>(uval & 0xFF);
        shadow_out_[offset + 1] = static_cast<uint8_t>((uval >>  8) & 0xFF);
        shadow_out_[offset + 2] = static_cast<uint8_t>((uval >> 16) & 0xFF);
        shadow_out_[offset + 3] = static_cast<uint8_t>((uval >> 24) & 0xFF);
    }

    /**
     * Write a 16-bit signed value at byte offset in the output shadow.
     */
    void write_i16(std::size_t offset, int16_t value) 
    {
        write_u16(offset, static_cast<uint16_t>(value));
    }

    /**
     * Write a single byte at offset in the output shadow.
     */
    void write_u8(std::size_t offset, uint8_t value) 
    {
        shadow_out_[offset] = value;
    }

    // Read helpers (shadow_in → application) ───────────────────────────────────────────────

    /**
     * Read a 16-bit unsigned value from the input shadow at byte offset.
     */
    uint16_t read_u16(std::size_t offset) const 
    {
        return static_cast<uint16_t>(shadow_in_[offset]) |
               (static_cast<uint16_t>(shadow_in_[offset + 1]) << 8);
    }

    /**
     * Read a 32-bit signed value from the input shadow at byte offset.
     */
    int32_t read_i32(std::size_t offset) const 
    {
        uint32_t uval = static_cast<uint32_t>(shadow_in_[offset])             |
                        (static_cast<uint32_t>(shadow_in_[offset + 1]) <<  8) |
                        (static_cast<uint32_t>(shadow_in_[offset + 2]) << 16) |
                        (static_cast<uint32_t>(shadow_in_[offset + 3]) << 24);
        return static_cast<int32_t>(uval);
    }

    /**
     * Read a 16-bit signed value from the input shadow at byte offset.
     */
    int16_t read_i16(std::size_t offset) const 
    {
        return static_cast<int16_t>(read_u16(offset));
    }

    /**
     * Read a single byte from the input shadow at offset.
     */
    uint8_t read_u8(std::size_t offset) const 
    {
        return shadow_in_[offset];
    }

    /**
     * Read a u16 back from the "output" shadow (not the input shadow).
     * Used by DriveHAL for read-modify-write operations on the control word.
     */
    uint16_t read_ctrl_word() const 
    {
        return static_cast<uint16_t>(shadow_out_[DRIVE_CTRL_WORD_OFFSET]) |
               (static_cast<uint16_t>(shadow_out_[DRIVE_CTRL_WORD_OFFSET + 1]) << 8);
    }

    // Raw shadow access (used by tests only) ───────────────────────────────────────────────
    std::array<uint8_t, FRAME_SIZE>& shadow_out() { return shadow_out_; }

    const std::array<uint8_t, FRAME_SIZE>& shadow_in() const { return shadow_in_; }

private:
    BusSimulator& bus_;
    std::array<uint8_t, FRAME_SIZE> shadow_out_;
    std::array<uint8_t, FRAME_SIZE> shadow_in_;
};

} // namespace neura
