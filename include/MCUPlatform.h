/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// contains routines for handling microcontroller platform detection

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
#include <Arduino.h>
#include "type_traits.h"
#include "DependentFalse.h"

// comment this out to disable sram cache support
#ifdef ARDUINO_AVR_ATmega1284
#define PACKED_ATTRIBUTE
#else
#define PACKED_ATTRIBUTE __attribute__((packed))
#endif
#ifdef __AVR__
using int24_t = __int24;
using uint24_t = __uint24;
#endif


template<typename E>
constexpr bool isValidPin(E pin) noexcept {
    return static_cast<int>(pin) < static_cast<int>(E::Count) &&
           static_cast<int>(pin) >= 0;
}
template<auto pin>
constexpr bool isValidPin_v = isValidPin(pin);
constexpr unsigned long long int operator "" _KB(unsigned long long int value) noexcept { return value * 1024; }
constexpr unsigned long long int operator "" _MB(unsigned long long int value) noexcept { return value * 1024 * 1024; }
constexpr unsigned long long int operator "" _KHz(unsigned long long int value) noexcept { return value * 1000; }
constexpr unsigned long long int operator "" _MHz(unsigned long long int value) noexcept { return value * 1000 * 1000; }
static_assert(2_KHz == 2'000);
static_assert(2_MHz == 2'000'000);
static_assert(20_MHz == 20'000'000);

static constexpr byte BitMaskTable_Byte[8] {
        0b0000'0001,
        0b0000'0010,
        0b0000'0100,
        0b0000'1000,
        0b0001'0000,
        0b0010'0000,
        0b0100'0000,
        0b1000'0000,
};

constexpr byte pow2(byte value) noexcept {
    if (value == 0) {
        return 1;
    } else {
        return pow2(value - 1) * 2;
    }
}

static constexpr byte numberOfBitsForCount(uint16_t count) noexcept {
    switch (count) {
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        case 256: return 8;
        case 512: return 9;
        case 1024: return 10;
        case 2048: return 11;
        case 4096: return 12;
        case 8192: return 13;
        case 16384: return 14;
        case 32768: return 15;
        default: return 0;
    }
}
static constexpr byte getNumberOfBitsForNumberOfEntries(uint16_t count) noexcept { return numberOfBitsForCount(count); }

static_assert(getNumberOfBitsForNumberOfEntries(512/4) == 7);
static_assert(getNumberOfBitsForNumberOfEntries(256/4) == 6);

template<byte numBits>
using ClosestBitValue_t = conditional_t<numBits <= 8, byte,
                                        conditional_t<numBits <= 16, uint16_t,
                                        conditional_t<numBits <= 32, uint32_t, uint64_t>>>;

static_assert(is_same_v<ClosestBitValue_t<1>, ClosestBitValue_t<4>>);
static_assert(is_same_v<ClosestBitValue_t<4>, byte>);
static_assert(is_same_v<ClosestBitValue_t<10>, uint16_t>);
static_assert(!is_same_v<ClosestBitValue_t<10>, ClosestBitValue_t<4>>);


enum class TargetMCU {
    ATmega1284p_Type1,
    ATmega1284p_Type2,
    Unknown,
};
class MCUConfiguration final {
public:
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t psramSpeedCap
    ) noexcept : sramAmount_(sramSize),
                 ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                 psramSpeedCap_(psramSpeedCap > 33_MHz ? 33_MHz : psramSpeedCap) { }

    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
    [[nodiscard]] constexpr auto runPSRAMAt() const noexcept { return psramSpeedCap_; }
private:
    uint32_t sramAmount_;
    uint32_t ioExpanderPeripheralSpeed_;
    uint32_t psramSpeedCap_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {
        0,
        10_MHz,
        8_MHz
};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p_Type1> = {
        16_KB,
        10_MHz,
        5_MHz // due to the current design, we have to run the psram at 5 Mhz
};

template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p_Type2> = {
        16_KB,
        10_MHz,
        10_MHz // due to the current design, we have to run the psram at 5 Mhz
};

class TargetBoard {
public:
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    [[nodiscard]] static constexpr TargetMCU getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
#ifdef CHIPSET_TYPE1
        return TargetMCU::ATmega1284p_Type1;
#elif defined(CHIPSET_TYPE2)
        return TargetMCU::ATmega1284p_Type2;
#else
        return TargetMCU::Unknown;
#endif
#else
        return TargetMCU::Unknown;
#endif
    }
    template<TargetMCU mcu>
    [[nodiscard]] static constexpr auto targetMCUIs() noexcept { return getMCUTarget() == mcu; }
    template<TargetMCU ... rest>
    [[nodiscard]] static constexpr auto targetMCUIsOneOfThese() noexcept {
        return (targetMCUIs<rest>() || ...);
    }
    [[nodiscard]] static constexpr auto onAtmega1284p_Type1() noexcept { return targetMCUIs<TargetMCU::ATmega1284p_Type1>(); }
    [[nodiscard]] static constexpr auto onAtmega1284p_Type2() noexcept { return targetMCUIs<TargetMCU::ATmega1284p_Type2>(); }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return targetMCUIsOneOfThese<TargetMCU::ATmega1284p_Type1, TargetMCU::ATmega1284p_Type2>(); }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return targetMCUIs<TargetMCU::Unknown>(); }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
    [[nodiscard]] static constexpr auto runPSRAMAt() noexcept { return BoardDescription<getMCUTarget()>.runPSRAMAt(); }
public:
    TargetBoard() = delete;
    ~TargetBoard() = delete;
    TargetBoard(const TargetBoard&) = delete;
    TargetBoard(TargetBoard&&) = delete;
    TargetBoard& operator=(const TargetBoard&) = delete;
    TargetBoard& operator=(TargetBoard&&) = delete;
};

static_assert(!TargetBoard::onUnknownTarget(), "ERROR: Target Board has not been defined, please define to continue");
static_assert(TargetBoard::getSRAMAmountInBytes() >= 16_KB, "ERROR: Less than 16kb of sram is not allowed!");

/**
 * @brief The backing design of the registers within the chipset that are 32-bits in width
 */
union SplitWord16 {
    explicit constexpr SplitWord16(uint16_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord16(uint8_t lower, uint8_t upper) noexcept : bytes{lower, upper} { }
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return bytes[0]; }
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return bytes[1]; }
    uint16_t wholeValue_ = 0;
    uint8_t bytes[sizeof(uint16_t) / sizeof(uint8_t)];
};
union SplitWord32 {
    // adding this dropped program size by over 500 bytes!
    explicit constexpr SplitWord32(uint32_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord32(uint16_t lower, uint16_t upper) noexcept : halves{lower, upper} {}
    constexpr SplitWord32(uint8_t lowest, uint8_t lower, uint8_t higher, uint8_t highest) noexcept : bytes{lowest, lower, higher, highest} {}
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getSignedRepresentation() const noexcept { return signedRepresentation_; }
    [[nodiscard]] constexpr auto getTargetPage() const noexcept { return static_cast<byte>(wholeValue_ >> 8); }
    [[nodiscard]] constexpr auto getMostSignificantByte() const noexcept { return static_cast<byte>(wholeValue_ >> 24); }
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return halves[0]; }
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return halves[1]; }
    void setLowerHalf(SplitWord16 value) noexcept { words_[0] = value; }
    void setUpperHalf(SplitWord16 value) noexcept { words_[1] = value; }
    uint32_t wholeValue_ = 0;
    int32_t signedRepresentation_;
    byte bytes[sizeof(uint32_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    SplitWord16 words_[sizeof(uint32_t) / sizeof(SplitWord16)];
    float floatingPointRepresentation_;
};
#endif //I960SXCHIPSET_MCUPLATFORM_H
