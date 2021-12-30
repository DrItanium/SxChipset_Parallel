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
//
// Created by jwscoggins on 10/17/21.
//

#ifndef SXCHIPSET_SERIAL0INTERFACE_H
#define SXCHIPSET_SERIAL0INTERFACE_H

template<Address baseAddress, bool addressDebuggingAllowed, bool defaultAddressDebuggingModeTo = false>
class Serial0Interface {
public:
    static constexpr auto StartAddress = baseAddress;
    static constexpr SplitWord32 StartAddressSplit { StartAddress };
    static constexpr auto StartPage = StartAddressSplit.getTargetPage();
    static constexpr auto EndAddress = StartAddress + 0x100;
    static constexpr SplitWord32 EndAddressSplit{EndAddress};
    static constexpr auto EndPage = EndAddressSplit.getTargetPage();
    static constexpr auto SectionID = StartAddressSplit.getMostSignificantByte();
    static constexpr auto AddressDebuggingAllowed = addressDebuggingAllowed;
    enum class Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(ConsoleIO),
        TwoByteEntry(ConsoleFlush),
        //FourByteEntry(ConsoleTimeout),
        //TwoByteEntry(ConsoleRXBufferSize),
        //TwoByteEntry(ConsoleTXBufferSize),
        //FourByteEntry(ChipsetClockSpeed),
        //TwoByteEntry(CacheLineCount),
        //TwoByteEntry(CacheLineSize),
        //TwoByteEntry(NumberOfCacheWays),
        TwoByteEntry(TriggerInterrupt),
        FourByteEntry(AddressDebuggingFlag),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        ConsoleIO = ConsoleIO0,
        ConsoleFlush = ConsoleFlush0,
        TriggerInterrupt = TriggerInterrupt0,
        AddressDebuggingFlag = AddressDebuggingFlag00,
        // we ignore the upper half of the register but reserve it to make sure
    };
    static_assert(static_cast<int>(Registers::End) < 0x100);
public:
    Serial0Interface() = delete;
    ~Serial0Interface() = delete;
private:
    static inline uint16_t handleFirstPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::ConsoleIO:
                return Serial.read();
            case Registers::AddressDebuggingFlag:
                if constexpr (AddressDebuggingAllowed) {
                    return static_cast<uint16_t>(enableAddressDebugging_);
                }
            default:
                return 0;
        }
    }
    static inline void handleFirstPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::TriggerInterrupt:
                break;
            case Registers::ConsoleFlush:
                Serial.flush();
                break;
            case Registers::ConsoleIO:
                Serial.write(static_cast<char>(value.getWholeValue()));
                break;
            case Registers::AddressDebuggingFlag:
                if constexpr (AddressDebuggingAllowed) {
                    enableAddressDebugging_ = (value.getWholeValue() != 0);
                }
                break;
            default:
                break;
        }
    }
public:
    static void begin() noexcept {
        // this is done ahead of time
        Serial.println(F("CONSOLE UP!"));
    }

    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        return handleFirstPageRegisterReads(offset, lss);
    }

    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        handleFirstPageRegisterWrites(offset, lss, value);
    }
    static bool addressDebuggingEnabled() noexcept { return AddressDebuggingAllowed && enableAddressDebugging_; }
private:
    // 257th char is always zero and not accessible, prevent crap from going beyond the cache
    static inline bool enableAddressDebugging_ = defaultAddressDebuggingModeTo;
};
#endif //SXCHIPSET_SERIAL0INTERFACE_H
