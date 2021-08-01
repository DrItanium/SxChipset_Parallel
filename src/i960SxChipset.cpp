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
#include <Wire.h>
#include "Pinout.h"

#include "ProcessorSerializer.h"
#include "MemoryThing.h"
#include "MemoryMappedFileThing.h"
#include "SDCardFileSystemInterface.h"
#include "CoreChipsetFeatures.h"
#include "TFTShieldThing.h"
#define ALLOW_SRAM_CACHE
//#define DEN_CONNECTED_TO_INTERRUPT

//bool displayReady = false;
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------

CoreChipsetFeatures chipsetFunctions(0);



class ROMTextSection : public MemoryMappedFile {
public:
    static constexpr Address ROMStart = 0;
    static constexpr Address ROMEnd = 0x2000'0000;
    static constexpr Address ROMMask = ROMEnd - 1;
    using Parent = MemoryMappedFile;
public:
    ROMTextSection() noexcept : Parent(ROMStart, ROMEnd, ROMEnd - 1, "boot.rom", FILE_READ){ }
    ~ROMTextSection() override = default;
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        return input & ROMMask;
    }
    [[nodiscard]] bool
    respondsTo(Address address) const noexcept override {
        return address < Parent::getFileSize();
    }
    using MemoryThing::respondsTo;
};

/// @todo add support for the boot data section that needs to be copied into ram by the i960 on bootup
class ROMDataSection : public MemoryMappedFile {
public:
    // two clusters are held onto at a time
    static constexpr Address ROMStart = 0x2000'0000;
    static constexpr Address ROMEnd = 0x8000'0000;
    static constexpr Address DataSizeMax = ROMEnd - ROMStart;
    using Parent = MemoryMappedFile;
public:
    ROMDataSection() noexcept : Parent(ROMStart, ROMEnd, DataSizeMax, "boot.dat", FILE_READ) { }
    ~ROMDataSection() override = default;
};
class RAMFile : public MemoryMappedFile {
    //<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()>
public:
    static constexpr Address MaxRamSize = 32 * 0x0100'0000; // 32 Memory Spaces or 512 Megabytes
    static constexpr auto RamMask = MaxRamSize - 1;
    using Parent = MemoryMappedFile;
    explicit RAMFile(Address baseAddress) noexcept : Parent(baseAddress, baseAddress + MaxRamSize, MaxRamSize, "ram.bin", FILE_WRITE) { }
    ~RAMFile() override = default;
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        // in this case, we want relative offsets
        return input & RamMask;
    }
    void begin() noexcept override {
        Parent::begin();
        if (Parent::getFileSize() != MaxRamSize) {
            signalHaltState(F("RAM.BIN MUST BE 512 MEGS IN SIZE!"));
        }
    }
};
DisplayThing displayCommandSet(0x200);
constexpr Address RAMStart = 0x8000'0000;
// this file overlays with the normal psram chip so any memory not accounted for goes to sdcard
RAMFile ram(RAMStart); // we want 4k but laid out for multiple sd card clusters, we can hold onto 8 at a time
ROMTextSection rom;
ROMDataSection dataRom;
SDCardFilesystemInterface fs(0x300);

// list of io memory devices to walk through
MemoryThing* things[] {
        &ram,
        &rom,
        &dataRom,
        &chipsetFunctions,
        &displayCommandSet,
        &fs,
};




// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

MemoryThing* theThing = nullptr;
// we only have a single 128kb cache chip
union TaggedAddress {
    constexpr explicit TaggedAddress(Address value = 0)  noexcept : base(value) { }
    constexpr auto getTagIndex() const noexcept { return tagIndex; }
    Address base;
    struct {
        uint8_t lowest4 : 4;
        uint8_t tagIndex : 8;
        Address rest : 20;
    };
};
constexpr uint8_t computeTagIndex(Address address) noexcept {
    return TaggedAddress(address).getTagIndex();
}
constexpr Address computeL2TagIndex(Address address) noexcept {
    // we don't care about the upper most bit because the SRAM cache isn't large enough
    return (address & 0xFFFF'FFF0) << 1;
}
constexpr bool EnableDebuggingCompileTime = false;
class CacheEntry {
public:
    CacheEntry() noexcept { };
    [[nodiscard]] constexpr bool valid() const noexcept { return valid_; }
    [[nodiscard]] constexpr bool isDirty() const noexcept { return dirty_; }
    static void invalidateAllEntries() noexcept {
#ifdef ALLOW_SRAM_CACHE
        // we need to walk through all of the sram cache entries, committing all entries back to the backing store
        for (uint32_t i = 0; i < 128_KB; i += 32) {
            CacheEntry target(i); // load from the cache and purge the hell out of it
            target.invalidate(); // try and invalidate it as well
        }
#endif
    }
private:
#ifdef ALLOW_SRAM_CACHE
    /**
     * @brief Construct a cache entry from the SRAM cache
     * @param tag The address to use to pull from the SRAM cache
     */
    explicit CacheEntry(Address tag) noexcept {
        absorbEntryFromSRAMCache(tag);
    }
    void absorbEntryFromSRAMCache(Address newTag) noexcept {
        SplitWord32 translation(computeL2TagIndex(newTag));
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(0x03);
        SPI.transfer(translation.bytes[2]);
        SPI.transfer(translation.bytes[1]);
        SPI.transfer(translation.bytes[0]); // aligned to 32-byte boundaries
        SPI.transfer(backingStorage, 32);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        // and we are done!
    }
private:
    void commitToSRAM() noexcept {
        SplitWord32 translation(computeL2TagIndex(tag));
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(0x02);
        SPI.transfer(translation.bytes[2]);
        SPI.transfer(translation.bytes[1]);
        SPI.transfer(translation.bytes[0]); // aligned to 32-byte boundaries
        SPI.transfer(backingStorage, 32); // this will garbage out things by design
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
    }
#endif
public:
    void reset(Address newTag, MemoryThing& thing) noexcept {
#ifdef ALLOW_SRAM_CACHE
        // commit what we currently have in this object to sram cache (this could be invalid but it is important!)
        // at no point can the cache ever be manipulated outside of this method
        commitToSRAM();
        // pull the new tag's address out of sram and do some work with it
        absorbEntryFromSRAMCache(newTag);
        if (matches(newTag)) {
            // we got a match to just return
            return;
        }
#endif
        // no match so pull the data in from main memory
        if (valid() && isDirty()) {
            // just do the write out to disk to save time
            backingThing->write(tag, reinterpret_cast<byte*>(data), sizeof(data));
        }
        valid_ = true; // always set this
        dirty_ = false;
        tag = newTag;
        backingThing = &thing;
        thing.read(tag, reinterpret_cast<byte*>(data), sizeof (data));
    }
    void invalidate() noexcept {
        if (valid() && isDirty()) {
            backingThing->write(tag, reinterpret_cast<byte*>(data), sizeof(data));
            valid_ = false;
            dirty_ = false;
            tag = 0;
            backingThing = nullptr;
            //unused = 0; // make sure that this is correctly purged
        }
    }
    [[nodiscard]] constexpr bool matches(Address addr) const noexcept { return valid() && (tag == addr); }
    [[nodiscard]] const SplitWord16& get(byte offset) const noexcept { return data[offset & 0b111]; }
    void set(byte offset, LoadStoreStyle style, SplitWord16 value) noexcept {
        dirty_ = true;
        switch (auto& target = data[offset & 0b111];style) {
            case LoadStoreStyle::Full16:
                target.wholeValue_ = value.wholeValue_;
                break;
            case LoadStoreStyle::Lower8:
                target.bytes[0] = value.bytes[0];
                break;
            case LoadStoreStyle::Upper8:
                target.bytes[1] = value.bytes[1];
                break;
            default:
                signalHaltState(F("BAD LOAD STORE STYLE FOR SETTING A CACHE LINE"));
                break;
        }
    }
private:
    union {
        // align to 32-bytes to make sure that it perfectly aligns to pages in the secondary level cache
        byte backingStorage[32] = { 0 };
        struct {
            bool valid_;
            bool dirty_;
            MemoryThing* backingThing; // 2 bytes
            Address tag; // 4 bytes
            SplitWord16 data[8]; // 16 bytes
            //uint64_t unused; // 8 bytes
        };
    };
};
static_assert(sizeof(CacheEntry) == 32);
CacheEntry entries[256];
// we have a second level cache of 1 megabyte in sram over spi
void invalidateGlobalCache() noexcept {
    // commit all entries back
    // walk through the SRAMCache and commit any entries which are empty
    for (auto& entry : entries) {
        entry.invalidate();
    }
    CacheEntry::invalidateAllEntries();
}
void setupPeripherals() {
    Serial.println(F("Setting up peripherals..."));
    //displayCommandSet.begin();
    //displayReady = true;
    rom.begin();
    dataRom.begin();
    ram.begin();
    // setup the bus things
    Serial.println(F("Done setting up peripherals..."));
}

#ifdef ALLOW_SRAM_CACHE
/**
 * @brief Just in case, purge the sram of data
 */
void purgeSRAMCache() noexcept {
    // 23lc1024s are in sequential by default :)
    digitalWrite(i960Pinout::CACHE_EN_, LOW);
    SPI.transfer(0xFF);
    digitalWrite(i960Pinout::CACHE_EN_, HIGH);
    constexpr uint32_t max = 128_KB; // only one SRAM chip on board
    Serial.println(F("CHECKING SRAM IS PROPERLY WRITABLE"));
    for (uint32_t i = 0; i < max; i += 32) {
        SplitWord32 translation(i);
        byte pagePurgeInstruction[36]{
                0x02,
                translation.bytes[2],
                translation.bytes[1],
                translation.bytes[0],
                1, 2, 3, 4, 5, 6, 7, 8,
                9, 10, 11, 12, 13, 14, 15, 16,
                17, 18, 19, 20, 21, 22, 23, 24,
                25, 26, 27, 28, 29, 30, 31, 32,
        };
        byte pageReadInstruction[36]{
                0x03,
                translation.bytes[2],
                translation.bytes[1],
                translation.bytes[0],
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
        };
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(pagePurgeInstruction, 36);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(pageReadInstruction, 36);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        for (int x = 4; x < 36; ++x) {
            auto a = pageReadInstruction[x];
            auto index = x - 3;
            if (a != index) {
                Serial.print(F("MISMATCH 0x"));
                Serial.print(index, HEX);
                Serial.print(F(" => 0x"));
                Serial.println(a, HEX);
            }
        }
    }
    Serial.println(F("SUCCESSFULLY CHECKED SRAM CACHE!"));
    Serial.println(F("PURGING SRAM CACHE!"));
    for (uint32_t i = 0; i < max; i+= 32) {
        SplitWord32 translation(i);
        byte pagePurgeInstruction[36] {
                0x02,
                translation.bytes[2],
                translation.bytes[1],
                translation.bytes[0],
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
        };
        byte pageReadInstruction[36]{
                0x03,
                translation.bytes[2],
                translation.bytes[1],
                translation.bytes[0],
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
        };
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(pagePurgeInstruction, 36);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(pageReadInstruction, 36);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        for (int x = 4; x < 36; ++x) {
            if (pageReadInstruction[x] != 0) {
                Serial.print(F("CHECK FAILURE!!!"));
            }
        }
    }
    Serial.println(F("DONE PURGING SRAM CACHE!"));
}
#endif
// the setup routine runs once when you press reset:
#ifdef DEN_CONNECTED_TO_INTERRUPT
volatile bool denTriggered = false;
void triggerDataEnable() noexcept {
    denTriggered = true;
}
#endif
void setup() {

    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::PSRAM_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::CACHE_EN_,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::SPI_OFFSET0,
              i960Pinout::SPI_OFFSET1,
              i960Pinout::SPI_OFFSET2,
              i960Pinout::Int0_);
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWriteBlock(HIGH,
                          i960Pinout::PSRAM_EN,
                          i960Pinout::SD_EN,
                          i960Pinout::CACHE_EN_,
                          i960Pinout::DISPLAY_EN,
                          i960Pinout::Ready,
                          i960Pinout::GPIOSelect,
                          i960Pinout::Int0_);
        digitalWriteBlock(LOW,
                          i960Pinout::SPI_OFFSET0,
                          i960Pinout::SPI_OFFSET1,
                          i960Pinout::SPI_OFFSET2);
        setupPins(INPUT,
                  i960Pinout::BLAST_,
                  i960Pinout::AS_,
                  i960Pinout::W_R_,
                  i960Pinout::DEN_,
                  i960Pinout::FAIL,
                  i960Pinout::BA1,
                  i960Pinout::BA2,
                  i960Pinout::BA3,
                  i960Pinout::BE0,
                  i960Pinout::BE1);
        //pinMode(i960Pinout::MISO, INPUT_PULLUP);
        SPI.begin();
#ifdef ALLOW_SRAM_CACHE
        purgeSRAMCache();
#endif
        theThing = &rom;
        fs.begin();
        chipsetFunctions.begin();
        Serial.println(F("i960Sx chipset bringup"));
        // purge the cache pages
#ifdef DEN_CONNECTED_TO_INTERRUPT
        attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), triggerDataEnable, FALLING);
#endif
        processorInterface.begin();
        setupPeripherals();
        delay(1000);
        Serial.println(F("i960Sx chipset brought up fully!"));

    }
    // at this point we have started execution of the i960
    // wait until we enter self test state
    while (DigitalPin<i960Pinout::FAIL>::isDeasserted());

    // now wait until we leave self test state
    while (DigitalPin<i960Pinout::FAIL>::isAsserted());
    // at this point we are in idle so we are safe to loaf around a bit
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
auto& getLine() noexcept {
    if constexpr (EnableDebuggingCompileTime) {
        Serial.println(F("getLine() {"));
    }
    auto address = processorInterface.getAlignedAddress();
    if constexpr (EnableDebuggingCompileTime) {
        auto tagIndex = computeTagIndex(address);
        Serial.print(F("ADDRESS: 0x"));
        Serial.println(address, HEX);
        Serial.print(F("TAG INDEX: 0x"));
        Serial.println(tagIndex, HEX);
    }
    auto& theEntry = entries[computeTagIndex(address)];
    if (!theEntry.matches(address)) {
        theEntry.reset(address, *theThing);
    }
    if constexpr (EnableDebuggingCompileTime) {
        Serial.println(F("}"));
    }
    return theEntry;
}

void loop() {
    do {
        if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
            signalHaltState(F("CHECKSUM FAILURE!"));
        }
        // wait until den is triggered via interrupt, we could even access the base address of the memory transaction
#ifdef DEN_CONNECTED_TO_INTERRUPT
        while (!denTriggered);
        denTriggered = false;
#else
        while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
#endif
        // keep processing data requests until we
        // when we do the transition, record the information we need
        processorInterface.newDataCycle();
        // W\~R is latched on chip for the entire duration of the transaction,
        // there is no reason to not cache it here
        auto isReadOperation = DigitalPin<i960Pinout::W_R_>::isAsserted();
        if (!theThing->respondsTo(processorInterface.getAddress(), LoadStoreStyle::Full16)) {
            theThing = getThing(processorInterface.getAddress(), LoadStoreStyle::Full16);
            // the only time that this is an issue is if we have to grab a new thing so only check
            // validity on getting a new thing
            if (!theThing) {
                // halt here because we've entered into unmapped memory state
                if (DigitalPin<i960Pinout::W_R_>::isAsserted()) {
                    Serial.print(F("UNMAPPED READ FROM 0x"));
                } else {
                    Serial.print(F("UNMAPPED WRITE OF 0x"));
                    // expensive but something has gone horribly wrong anyway so whatever!
                    Serial.print(processorInterface.getDataBits(), HEX);
                    Serial.print(F(" TO 0x"));

                }
                Serial.println(processorInterface.getAddress(), HEX);
                signalHaltState(F("UNMAPPED MEMORY REQUEST!"));
            }
        }
        auto signalDone = []() {
            auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            DigitalPin<i960Pinout::Ready>::pulse();
            return isBurstLast;
        };
        if (theThing->bypassesCache()) {
            if (isReadOperation) {
                do {
                    processorInterface.updateDataCycle();
                    auto address = processorInterface.getAddress();
                    auto style = processorInterface.getStyle();
                    processorInterface.setDataBits(theThing->read(address, style));
                } while (!signalDone());
            } else {
                // write
                do {
                    processorInterface.updateDataCycle();
                    Address burstAddress = processorInterface.getAddress();
                    LoadStoreStyle style = processorInterface.getStyle();
                    theThing->write(burstAddress, processorInterface.getDataBits(), style);
                } while (!signalDone());
            }
        } else {
            if (auto &theEntry = getLine(); isReadOperation) {
                do {
                    processorInterface.updateDataCycle();
                    auto result = theEntry.get(processorInterface.getBurstAddressBits()).getWholeValue();
                    processorInterface.setDataBits(result);
                } while (!signalDone());
            } else {
                do {
                    processorInterface.updateDataCycle();
                    SplitWord16 theBits(processorInterface.getDataBits());
                    theEntry.set(processorInterface.getBurstAddressBits(), processorInterface.getStyle(), theBits);
                } while (!signalDone());
            }
        }
    } while (true);
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
#if 0
    if (displayReady) {
        displayCommandSet.clearScreen();
        displayCommandSet.setCursor(0, 0);
        displayCommandSet.setTextSize(2);
        displayCommandSet.println(haltMsg);
    }
#endif
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}

MemoryThing*
getThing(Address address, LoadStoreStyle style) noexcept {
    for (auto *currentThing : things) {
        if (currentThing->respondsTo(address, style)) {
            return currentThing;
        }
    }
    return nullptr;
}
SdFat SD;
/// @todo Eliminate after MightyCore update
#if __cplusplus >= 201402L

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}

void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}

#endif // end language is C++14 or greater
