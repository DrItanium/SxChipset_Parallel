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

#ifndef SXCHIPSET_RTCINTERFACE_H
#define SXCHIPSET_RTCINTERFACE_H
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

#include "Pinout.h"

template<Address baseAddress>
class RTCInterface {
public:
    static constexpr auto StartAddress = baseAddress;
    static constexpr SplitWord32 StartAddressSplit { StartAddress };
    static constexpr auto EndAddress = StartAddress + 0x100;
    static constexpr SplitWord32 EndAddressSplit{EndAddress};
    static constexpr auto StartPage = StartAddressSplit.getTargetPage();
    static constexpr auto EndPage = EndAddressSplit.getTargetPage();
    static constexpr auto SectionID = StartAddressSplit.getMostSignificantByte();
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
        TwoByteEntry(Available),
        TwoByteEntry(NowRequest),
        TwoByteEntry(Seconds),
        TwoByteEntry(Minutes),

        TwoByteEntry(Hours),
        TwoByteEntry(Day),

        TwoByteEntry(Month),
        TwoByteEntry(Year),

        TwoByteEntry(DayOfTheWeek),
        TwoByteEntry(Reserved),
        FourByteEntry(Secondstime),
        FourByteEntry(Unixtime),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Available = Available0,
        NowRequest = NowRequest0,
        UnixtimeLower = Unixtime00,
        UnixtimeUpper = Unixtime10,
        SecondstimeLower = Secondstime00,
        SecondstimeUpper = Secondstime10,
        Seconds = Seconds0,
        Minutes = Minutes0,
        Hours = Hours0,
        Day = Day0,
        Month = Month0,
        Year = Year0,
        DayOfTheWeek = DayOfTheWeek0,
    };
public:
    RTCInterface() = delete;
    ~RTCInterface() = delete;
    static void begin() noexcept {
        if constexpr (!TargetBoard::onAtmega1284p_Type2()) {
            rtcUp_ = rtc_.begin();
            if (!rtcUp_) {
                Serial.println(F("NO RTC FOUND...DISABLING"));
            } else {
                Serial.println(F("RTC FOUND... CHECKING"));
                if (!rtc_.initialized() || rtc_.lostPower()) {
                    Serial.println(F("RTC is NOT initialized, setting time from sketch compile"));
                    // not the most accurate but good enough
                    rtc_.adjust(DateTime(F(__DATE__), F(__TIME__)));
                }
                // make sure that all timers are shutoff on startup
                rtc_.deconfigureAllTimers();
                DateTime now = rtc_.now();
                Serial.print(now.year(), DEC);
                Serial.print(F("/"));
                Serial.print(now.month(), DEC);
                Serial.print(F("/"));
                Serial.print(now.day(), DEC);
                Serial.print(F(" "));
                Serial.print(now.hour(), DEC);
                Serial.print(F(":"));
                Serial.print(now.minute(), DEC);
                Serial.print(F(":"));
                Serial.print(now.second(), DEC);
                Serial.println();

                rtc_.start();
            }
        }
    }
    static uint16_t read(uint8_t, uint8_t offset, LoadStoreStyle) noexcept {
        if constexpr (!TargetBoard::onAtmega1284p_Type2()) {
            switch (static_cast<Registers>(offset)) {
                case Registers::Seconds:
                    return static_cast<uint16_t>(now_.second());
                case Registers::Hours:
                    return static_cast<uint16_t>(now_.hour());
                case Registers::Minutes:
                    return static_cast<uint16_t>(now_.minute());
                case Registers::Day:
                    return static_cast<uint16_t>(now_.day());
                case Registers::Month:
                    return static_cast<uint16_t>(now_.month());
                case Registers::Year:
                    return static_cast<uint16_t>(now_.year());
                case Registers::DayOfTheWeek:
                    return static_cast<uint16_t>(now_.dayOfTheWeek());
                case Registers::Available:
                    return rtcUp_ ? 0xFFFF : 0;
                case Registers::UnixtimeLower:
                    return static_cast<uint16_t>(unixtime_);
                case Registers::UnixtimeUpper:
                    return static_cast<uint16_t>(unixtime_ >> 16);
                case Registers::SecondstimeLower:
                    return static_cast<uint16_t>(secondstime_);
                case Registers::SecondstimeUpper:
                    return static_cast<uint16_t>(secondstime_ >> 16);
                default:
                    return 0;
            }
        } else {
            return 0;
        }
    }
    static void write(uint8_t, uint8_t offset, LoadStoreStyle, SplitWord16) noexcept {
        if constexpr (!TargetBoard::onAtmega1284p_Type2()) {
            switch (static_cast<Registers>(offset)) {
                case Registers::NowRequest: {
                    if (rtcUp_) {
                        now_ = rtc_.now();
                        unixtime_ = now_.unixtime();
                        secondstime_ = now_.secondstime();
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
    static constexpr auto available() noexcept { return rtcUp_; }
private:
    static inline RTC_PCF8523 rtc_;
    static inline bool rtcUp_ = false;
    static inline uint32_t unixtime_{0};
    static inline uint32_t secondstime_{0};
    static inline DateTime now_;
};

#endif //SXCHIPSET_RTCINTERFACE_H
