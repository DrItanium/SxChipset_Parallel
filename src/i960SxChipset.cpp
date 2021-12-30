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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <SdFat.h>
#include "Pinout.h"

#include "CacheEntry.h"
#include "DirectMappedCacheWay.h"
#include "TwoWayLRUCacheEntry.h"
#include "FourWayPseudoLRUEntry.h"
#include "EightWayPseudoLRUEntry.h"
#include "SixteenWayPseudoLRUEntry.h"
#include "EightWayRandPLRUEntry.h"
#include "EightWayTreePLRUEntry.h"
#include "SinglePoolCache.h"
#include "MultiCache.h"

#include "ProcessorSerializer.h"
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include "type_traits.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 16;
constexpr auto CompileInAddressDebuggingSupport = true;
constexpr auto AddressDebuggingEnabledOnStartup = true;
constexpr auto ValidateTransferDuringInstall = false;
/**
 * @brief When set to true, the interrupt lines the mcp23s17 provides are used to determine which bytes to read
 */
using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
using ConfigurationSpace = CoreChipsetFeatures<TheConsoleInterface,
        TheSDInterface,
        TheDisplayInterface,
        TheRTCInterface>;
// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
using FallbackMemory = SDCardAsRam<TheSDInterface >;
template<TargetMCU mcu> struct BackingMemoryStorage final { using Type = FallbackMemory; };

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;
constexpr auto computeCacheLineSize() noexcept { return 6; }
//using OnboardPSRAMBlock = ::
constexpr auto NumAddressBitsForPSRAMCache = 32;
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
constexpr auto CacheLineSize = computeCacheLineSize();
constexpr auto CacheSize = 8192;

//using L1Cache = CacheInstance_t<EightWayTreePLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
//using L1Cache = CacheInstance_t<EightWayLRUCacheWay, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
using L1Cache = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
L1Cache theCache;

//template<template<auto, auto, auto, typename> typename L,
//        byte NumberOfCaches,
//        uint16_t IndividualCacheSize,
//        byte CacheLineSize>
//using L1Cache = MultiCache<L, NumberOfCaches, IndividualCacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
//L1Cache<DirectMappedCacheWay, 11, 1024, 6> theCache;





template<bool inDebugMode>
[[nodiscard]] bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    if constexpr (inDebugMode) {
        Serial.println(F("\tTelling the i960 that the data lines are ready!"));
    }
    setDemuxAsReady();
    DigitalPin<i960Pinout::Ready_>::assertPin();
    if constexpr (inDebugMode) {
        Serial.println(F("\tWAITING FOR IN_TRANSACTION TO GO HIGH OR BURST_NEXT TO GO LOW"));
    }
    while (DigitalPin<i960Pinout::IN_TRANSACTION_>::isAsserted() && DigitalPin<i960Pinout::BURST_NEXT_>::isDeasserted());
    bool outcome = DigitalPin<i960Pinout::IN_TRANSACTION_>::isDeasserted();
    if constexpr (inDebugMode) {
        if (outcome) {
            Serial.println(F("\t\tTRANSACTION COMPLETE"));
        } else {
            Serial.println(F("\t\tBURST NEXT"));
        }
    }
    DigitalPin<i960Pinout::Ready_>::deassertPin();
    return outcome;
}
template<bool inDebugMode>
inline void waitForCycleUnlock() noexcept {
    if constexpr (inDebugMode) {
        Serial.println(F("\tWAITING FOR CYCLE UNLOCK"));
    }
    while (DigitalPin<i960Pinout::DO_CYCLE_>::isDeasserted()) {
        if (DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::read() == LOW) {
            signalHaltState(F("CHECKSUM FAILURE!"));
        }
    }
    if constexpr (inDebugMode) {
        Serial.println(F("\tCYCLE UNLOCK COMPLETE!"));
    }
}
constexpr auto IncrementAddress = true;
constexpr auto LeaveAddressAlone = false;
// while the i960 does not allow going beyond 8 words, we can use the number of words cached in all cases to be safe
constexpr byte MaximumNumberOfWordsTransferrableInASingleTransaction = decltype(theCache)::NumWordsCached;
inline void displayRequestedAddress() noexcept {
    auto address = ProcessorInterface::getAddress();
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(address, HEX);
}

template<bool inDebugMode>
inline void fallbackBody() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // fallback, be consistent to make sure we don't run faster than the i960
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        for (;;) {
            waitForCycleUnlock<inDebugMode>();
            // need to introduce some delay
            ProcessorInterface::setDataBits<inDebugMode>(0);
            if (informCPU<inDebugMode>()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        for (;;) {
            waitForCycleUnlock<inDebugMode>();
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            __builtin_avr_nops(4);
            // need to introduce some delay
            if (informCPU<inDebugMode>()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode>
inline void handleMemoryInterface() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // okay we are dealing with the psram chips
    // now take the time to compute the cache offset entries
    if (auto& theEntry = theCache.getLine(); ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
        // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            waitForCycleUnlock<inDebugMode>();
            auto outcome = theEntry.get(i);
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tRead the value: 0x"));
                Serial.println(outcome, HEX);
            }
            // Only pay for what we need even if it is slower
            ProcessorInterface::setDataBits<inDebugMode>(outcome);
            if (informCPU<inDebugMode>()) {
                break;
            }
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
        // far as we can go with how the Sx works!

        // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            waitForCycleUnlock<inDebugMode>();
            auto bits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tWriting the value: 0x"));
                Serial.println(bits.getWholeValue(), HEX);
            }
            theEntry.set(i, ProcessorInterface::getStyle(), bits);
            if (informCPU<inDebugMode>()) {
                break;
            }
            // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
            // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode, typename T>
inline void handleExternalDeviceRequest() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // with burst transactions in the core chipset, we do not have access to a cache line to write into.
    // instead we need to do the old style infinite iteration design
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        for(;;) {
            waitForCycleUnlock<inDebugMode>();
            auto result = T::read(ProcessorInterface::getPageIndex(),
                                  ProcessorInterface::getPageOffset(),
                                  ProcessorInterface::getStyle());
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tRead Value: 0x"));
                Serial.println(result, HEX);
            }
            ProcessorInterface::setDataBits<inDebugMode>(result);
            if (informCPU<inDebugMode>()) {
                break;
            }
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        for (;;) {
            waitForCycleUnlock<inDebugMode>();
            auto dataBits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(dataBits.getWholeValue(), HEX);
            }
            T::write(ProcessorInterface::getPageIndex(),
                     ProcessorInterface::getPageOffset(),
                     ProcessorInterface::getStyle(),
                     dataBits);
            if (informCPU<inDebugMode>()) {
                break;
            }
            // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
            // so you may still be reading the previous i960 cycle state!
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    }
}

template<bool inDebugMode>
inline void invocationBody() noexcept {
    // wait for the management engine to tell us that we are in a transaction
    Serial.println(F("\tWAITING FOR TRANSACTION START"));
    while (DigitalPin<i960Pinout::IN_TRANSACTION_>::isDeasserted()) {
        if (DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::read() == LOW) {
            signalHaltState(F("CHECKSUM FAILURE!"));
        }
    }
    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    ProcessorInterface::newDataCycle<inDebugMode, decltype(theCache)::CacheEntryMask>();
}
template<bool allowAddressDebuggingCodePath>
void doInvocationBody() noexcept {
    if constexpr (allowAddressDebuggingCodePath) {
        if (TheConsoleInterface::addressDebuggingEnabled())  {
            invocationBody<true>();
        } else {
            invocationBody<false>();
        }
    } else {
        invocationBody<false>();
    }
}
void installBootImage() noexcept {

    // okay now we need to actually open boot.system and copy it into the ramBlock
    if (!SD.exists(const_cast<char*>("boot.sys"))) {
        // delete the file and start a new
        signalHaltState(F("Could not find file \"boot.sys\"!"));
    }
    if (auto theFile = SD.open("boot.sys", FILE_READ); !theFile) {
        signalHaltState(F("Could not open \"boot.sys\"! SD CARD may be corrupt?")) ;
    } else {
            // okay we were successful in opening the file, now copy the image into psram
        Address size = theFile.size();
        Serial.println(F("TRANSFERRING BOOT.SYS TO RAM"));
        static constexpr auto CacheSize = theCache.getCacheSize();
        //static constexpr auto CacheSize = ::CacheSize;
        auto *storage = theCache.viewAsStorage();
        if constexpr (ValidateTransferDuringInstall) {
            static constexpr auto RealCacheSize = CacheSize / 2;
            byte* storage0 = storage;
            byte* storage1 = storage + (RealCacheSize);
            for (Address addr = 0; addr < size; addr += RealCacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage0, RealCacheSize);
                if (numRead < 0) {
                    // something wen't wrong so halt at this point
                    SD.errorHalt();
                }
                (void) BackingMemoryStorage_t ::write(addr, storage0, numRead);
                (void) BackingMemoryStorage_t ::read(addr, storage1, numRead);
                // now read back the contents into the second buffer
                for (auto i = 0; i < numRead; ++i) {
                    auto a = storage0[i];
                    auto b = storage1[i];
                    if (a != b) {
                        Serial.print(F("MISMATCH WANTED 0x"));
                        Serial.print(a, HEX);
                        Serial.print(F(" BUT GOT 0x"));
                        Serial.println(b, HEX);
                    }
                }

                Serial.print(F("."));
            }
        } else {
            // use the cache as a buffer since it won't be in use at this point in time
            for (Address addr = 0; addr < size; addr += CacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage, CacheSize);
                if (numRead < 0) {
                    // something wen't wrong so halt at this point
                    SD.errorHalt();
                }
                (void) BackingMemoryStorage_t ::write(addr, storage, numRead);
                // now read back the contents into the upper half
                Serial.print(F("."));
            }
        }
        Serial.println();
        Serial.println(F("Transfer complete!"));
        // make sure we close the file before destruction
        theFile.close();
        // clear both caches to be on the safe side
        theCache.clear();
    }
}

using DispatchTable = BodyFunction[256];
DispatchTable lookupTable;
DispatchTable lookupTable_Debug;
void setupDispatchTable() noexcept {
    Serial.println(F("Setting up the initial lookup table"));
    for (auto& entry : lookupTable) {
        entry = fallbackBody<false>;
    }
    lookupTable[0] = handleMemoryInterface<false>;
    // only chipset type 1 has access to the full 64 megabytes
    lookupTable[1] = handleMemoryInterface<false>;
    lookupTable[2] = handleMemoryInterface<false>;
    lookupTable[3] = handleMemoryInterface<false>;
    lookupTable[TheRTCInterface ::SectionID] = handleExternalDeviceRequest<false, TheRTCInterface >;
    lookupTable[TheDisplayInterface ::SectionID] = handleExternalDeviceRequest<false, TheDisplayInterface>;
    lookupTable[TheSDInterface ::SectionID] = handleExternalDeviceRequest<false, TheSDInterface>;
    lookupTable[TheConsoleInterface :: SectionID] = handleExternalDeviceRequest<false, TheConsoleInterface >;
    lookupTable[ConfigurationSpace :: SectionID] = handleExternalDeviceRequest<false, ConfigurationSpace >;
    if constexpr (CompileInAddressDebuggingSupport) {
        for (auto &entry: lookupTable_Debug) {
            entry = fallbackBody<true>;
        }
        lookupTable_Debug[0] = handleMemoryInterface<true>;
        lookupTable_Debug[1] = handleMemoryInterface<true>;
        lookupTable_Debug[2] = handleMemoryInterface<true>;
        lookupTable_Debug[3] = handleMemoryInterface<true>;
        lookupTable_Debug[TheRTCInterface::SectionID] = handleExternalDeviceRequest<true, TheRTCInterface>;
        lookupTable_Debug[TheDisplayInterface::SectionID] = handleExternalDeviceRequest<true, TheDisplayInterface>;
        lookupTable_Debug[TheSDInterface::SectionID] = handleExternalDeviceRequest<true, TheSDInterface>;
        lookupTable_Debug[TheConsoleInterface::SectionID] = handleExternalDeviceRequest<true, TheConsoleInterface>;
        lookupTable_Debug[ConfigurationSpace::SectionID] = handleExternalDeviceRequest<true, ConfigurationSpace>;
    }
}

// the setup routine runs once when you press reset:
void setup() {
    // always do this first to make sure that we put the i960 into reset regardless of target
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    // get SPI setup ahead of time
    SPI.begin();
    DigitalPin<i960Pinout::DEMUX_EN>::configure();
    DigitalPin<i960Pinout::DEMUX_ID1>::configure();
    DigitalPin<i960Pinout::DEMUX_ID0>::configure();
    DigitalPin<i960Pinout::DEMUX_EN>::deassertPin();
    DigitalPin<i960Pinout::DEMUX_ID0>::assertPin();
    DigitalPin<i960Pinout::DEMUX_ID1>::assertPin();
    // all of these pins need to be pulled high
    // setup the pins that could be attached to an io expander separately
    DigitalPin<i960Pinout::MUX_SEL0>::configure();
    DigitalPin<i960Pinout::MUX_SEL0>::assertPin();
    DigitalPin<i960Pinout::MUX_CHAN0>::configure();
    DigitalPin<i960Pinout::MUX_CHAN1>::configure();
    DigitalPin<i960Pinout::IN_TRANSACTION_>::configure();
    DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::configure();
    DigitalPin<i960Pinout::DO_CYCLE_>::configure();
    DigitalPin<i960Pinout::BURST_NEXT_>::configure();
    DigitalPin<i960Pinout::DATA0>::configure();
    DigitalPin<i960Pinout::DATA1>::configure();
    DigitalPin<i960Pinout::DATA2>::configure();
    DigitalPin<i960Pinout::DATA3>::configure();
    DigitalPin<i960Pinout::DATA4>::configure();
    DigitalPin<i960Pinout::DATA5>::configure();
    DigitalPin<i960Pinout::DATA6>::configure();
    DigitalPin<i960Pinout::DATA7>::configure();
    DigitalPin<i960Pinout::DATA8>::configure();
    DigitalPin<i960Pinout::DATA9>::configure();
    DigitalPin<i960Pinout::DATA10>::configure();
    DigitalPin<i960Pinout::DATA11>::configure();
    DigitalPin<i960Pinout::DATA12>::configure();
    DigitalPin<i960Pinout::DATA13>::configure();
    DigitalPin<i960Pinout::DATA14>::configure();
    DigitalPin<i960Pinout::DATA15>::configure();
    theCache.begin();
    // purge the cache pages
    ConfigurationSpace::begin();
    Serial.println(F("i960Sx chipset bringup"));
    ProcessorInterface::begin();
    // now the i960 is in reset
    BackingMemoryStorage_t::begin();
    setupDispatchTable();
    installBootImage();
    delay(100);
    Serial.println(F("i960Sx chipset brought up fully!"));
    ProcessorInterface::pulli960OutOfReset();
    Wire.end();
    DDRA = 0;
    PORTA = 0;
    DDRC = 0;
    PORTC = 0;
    // at this point we have started execution of the i960
    // wait until we enter self test state
    while (DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::read() == LOW);

    // at this point we are in idle so we are safe to loaf around a bit
    // at this point, the i960 will request 32-bytes to perform a boot check sum on.
    // If the checksum is successful then execution will continue as normal
    // first set of 16-byte request from memory

    // on bootup we need to ignore the interrupt lines for now
    doInvocationBody<CompileInAddressDebuggingSupport>();
    doInvocationBody<CompileInAddressDebuggingSupport>();
    if (DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::read() == LOW) {
        signalHaltState(F("CHECKSUM FAILURE!"));
    } else {
        Serial.println(F("SYSTEM BOOT SUCCESSFUL!"));
    }
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// NOTE: Tw may turn out to be synthetic

void loop() {
    // by default, arduino's main has a boolean check inside of the loop for the serial event run function check
    // this is expensive and something I don't use. The infinite loop is for this purpose. It shaves off around 0.1usec in the worst case
    // and doesn't seem to impact performance in burst transactions

    for (;;) {
        doInvocationBody<CompileInAddressDebuggingSupport>();
    }
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    Serial.print(F("CHIPSET HALT: "));
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}
BodyFunction getNonDebugBody(byte index) noexcept {
    return lookupTable[index];
}
BodyFunction getDebugBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        return lookupTable_Debug[index];
    } else {
        return fallbackBody<true>;
    }
}

SdFat SD;
