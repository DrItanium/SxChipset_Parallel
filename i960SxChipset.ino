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
#include <libbonuspin.h>
#include <Fsm.h>
#include <Timer.h>
#include <SD.h>
#include <Wire.h>

#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <WiFiNINA.h> // Must be Adafruit's Fork
#include <Adafruit_BluefruitLE_SPI.h>
#include "Pinout.h"
#include "Device.h"
#include "RAM.h"
#include "SPIBus.h"
#include "IOExpanders.h"



/**
 * Normally generated by the linker as the value used for validation purposes
 * on system bootup. Since we are on a microcontroller, this can be done
 * dynamically. This is taken from start.ld in the SA/SB Reference manual's
 * Appendix D.
 */
constexpr auto computeCS1(uint32_t satPtr, uint32_t pcrbPtr, uint32_t startIP) noexcept {
	return - (satPtr + pcrbPtr + startIP);
}

Timer t;
Adafruit_ILI9341 tft(static_cast<int>(i960Pinout::DISPLAY_EN),
                     static_cast<int>(i960Pinout::DC));
Adafruit_BluefruitLE_SPI ble(static_cast<int>(i960Pinout::BLE_EN),
                             static_cast<int>(i960Pinout::BLE_IRQ),
                             static_cast<int>(i960Pinout::BLE_RST));
byte macAddress[6];
union WordEntry {
    byte bytes[2];
    uint16_t word;
};
constexpr auto OnBoardSRAMCacheSize = 8192 /* bytes */ / sizeof (WordEntry);
/**
 * @brief Allocate a portion of on board sram as accessible to the i960 without having to walk out onto the separate busses
 */
volatile WordEntry OnBoardSRAMCache[8192/sizeof(uint16_t)];


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
// Tr - Recovery State
// Tw - Wait State
// TChecksumFailure - Checksum Failure State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in 

// Ti -> Ti via no request
// Tr -> Ti via no request
// Tr -> Ta via request pending
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Tr after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// Tr -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
volatile bool asTriggered = false;
volatile bool denTriggered = false;
volatile uint32_t baseAddress = 0;
volatile bool performingRead = false;
constexpr auto NoRequest = 0;
constexpr auto NewRequest = 1;
constexpr auto ReadyAndBurst = 2;
constexpr auto NotReady = 3;
constexpr auto ReadyAndNoBurst = 4;
constexpr auto RequestPending = 5;
constexpr auto ToDataState = 6;
constexpr auto PerformSelfTest = 7;
constexpr auto SelfTestComplete = 8;
constexpr auto ChecksumFailure = 9;
void startingSystemTest() noexcept;
void systemTestPassed() noexcept;
void startupState() noexcept;
void systemTestState() noexcept;
void idleState() noexcept;
void doAddressState() noexcept;
void processDataRequest() noexcept;
void doRecoveryState() noexcept;
void enteringDataState() noexcept;
void enteringIdleState() noexcept;
State tStart(nullptr, startupState, nullptr);
State tSystemTest(nullptr, systemTestState, nullptr);
Fsm fsm(&tStart);
State tIdle(nullptr,
		idleState, 
		nullptr);
State tAddr([]() { asTriggered = false; }, 
		doAddressState, 
		nullptr);
State tData(enteringDataState, 
		processDataRequest, 
		nullptr);
State tRecovery(nullptr,
		doRecoveryState,
		nullptr);
State tChecksumFailure(nullptr, nullptr, nullptr);


void startupState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(PerformSelfTest);
	}
}
void systemTestState() noexcept {
    if (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
		fsm.trigger(SelfTestComplete);
	}
}
void onASAsserted() {
    asTriggered = true;
}
void onDENAsserted() {
    denTriggered = true;
}

void idleState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(NewRequest);
		}
	}
}
void doAddressState() noexcept {
	if (denTriggered) {
		fsm.trigger(ToDataState);
	}
}



void
enteringDataState() noexcept {
	// when we do the transition, record the information we need
	denTriggered = false;
	baseAddress = getAddress();
	performingRead = isReadOperation();
}
LoadStoreStyle getStyle() noexcept { return static_cast<LoadStoreStyle>(getByteEnableBits()); }

void
performWrite(Address address, uint16_t value) noexcept {
    Serial.print(F("Write 0x"));
    Serial.print(value, HEX);
    Serial.print(F(" to 0x"));
    Serial.println(address, HEX);
    /// @todo implement
}
uint16_t
performRead(Address address) noexcept {
    Serial.print(F("Read from 0x"));
    Serial.println(address, HEX);
    /// @todo implement
    return 0;
}
void processDataRequest() noexcept {
    auto burstAddress = getBurstAddress(baseAddress);
	if (performingRead) {
		setDataBits(performRead(burstAddress));
	} else {
	    performWrite(burstAddress, getDataBits());
	}
	// setup the proper address and emit this over serial
	auto blastPin = getBlastPin();
	DigitalPin<i960Pinout::Ready>::pulse();
	if (blastPin == LOW) {
		// we not in burst mode
		fsm.trigger(ReadyAndNoBurst);
	} 

}

void doRecoveryState() noexcept {
	if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
		fsm.trigger(ChecksumFailure);
	} else {
		if (asTriggered) {
			fsm.trigger(RequestPending);
		} else {
			fsm.trigger(NoRequest);
		}
	}
}


void setupBusStateMachine() noexcept {
	fsm.add_transition(&tStart, &tSystemTest, PerformSelfTest, nullptr);
	fsm.add_transition(&tSystemTest, &tIdle, SelfTestComplete, nullptr);
	fsm.add_transition(&tIdle, &tAddr, NewRequest, nullptr);
	fsm.add_transition(&tIdle, &tChecksumFailure, ChecksumFailure, nullptr);
	fsm.add_transition(&tAddr, &tData, ToDataState, nullptr);
	fsm.add_transition(&tData, &tRecovery, ReadyAndNoBurst, nullptr);
	fsm.add_transition(&tRecovery, &tAddr, RequestPending, nullptr);
	fsm.add_transition(&tRecovery, &tIdle, NoRequest, nullptr);
	fsm.add_transition(&tRecovery, &tChecksumFailure, ChecksumFailure, nullptr);
	fsm.add_transition(&tData, &tChecksumFailure, ChecksumFailure, nullptr);
}
//State tw(nullptr, nullptr, nullptr); // at this point, this will be synthetic
//as we have no concept of waiting inside of the mcu
void setupCPUInterface() {
    Serial.println(F("Setting up interrupts!"));
	setupPins(OUTPUT,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	digitalWriteBlock(HIGH,
			i960Pinout::Ready,
			i960Pinout::GPIOSelect,
			i960Pinout::Int0_);
	setHOLDPin(LOW);
	setLOCKPin(HIGH);
	setupPins(INPUT,
			i960Pinout::BLAST_,
			i960Pinout::AS_,
			i960Pinout::W_R_,
			i960Pinout::DEN_,
			i960Pinout::FAIL);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    Serial.println(F("Done setting up interrupts!"));
}
void setupIOExpanders() {
    Serial.println(F("Setting up IOExpanders!"));
	// at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
	// disabled. We can send out a single IOCON.HAEN enable message and all
	// should receive it. 
	// so do a begin operation on all chips (0b000)
	dataLines.begin(); 
	// set IOCON.HAEN on all chips
	dataLines.enableHardwareAddressPins();
	// now we have to refresh our on mcu flags for each io expander
	lower16.refreshIOCon();
	upper16.refreshIOCon();
	extraMemoryCommit.refreshIOCon();
	// now all devices tied to this ~CS pin have separate addresses
	// make each of these inputs
	lower16.writeGPIOsDirection(0xFFFF);
	upper16.writeGPIOsDirection(0xFFFF);
	dataLines.writeGPIOsDirection(0xFFFF);
	// set lower eight to inputs and upper eight to outputs
	extraMemoryCommit.writeGPIOsDirection(0x00FF);
	// then indirectly mark the outputs
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), OUTPUT, extraMemoryCommit);
	pinMode(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), OUTPUT, extraMemoryCommit);
    Serial.println(F("Done setting up io expanders!"));
}
void transferAddress(uint32_t address) {
    SPI.transfer(static_cast<uint8_t>(address >> 16));
    SPI.transfer(static_cast<uint8_t>(address >> 8));
    SPI.transfer(static_cast<uint8_t>(address));
}
void printMacAddress(byte mac[]) {
    for (int i = 5; i >= 0; --i) {
        if (mac[i] < 16) {
            Serial.print(F("0"));
        }
        Serial.print(mac[i], HEX);
        if (i > 0) {
            Serial.print(F(":"));
        }
    }
    Serial.println();
}
void setupWifi() noexcept {
    WiFi.setPins(static_cast<int>(i960Pinout::WIFI_EN),
                 static_cast<int>(i960Pinout::WIFI_BUSY),
                 static_cast<int>(i960Pinout::WIFI_RST),
                 -1, // disable
                 &SPI);

    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println(F("Communication with WiFi module failed!"));
        delay(1000);
    }
    auto fv = WiFi.firmwareVersion();
    Serial.println(fv);
    if (fv < "1.0.0") {
        Serial.println(F("Please upgrade the firmware"));
        while(1) {
            delay(10);
        }
    }
    Serial.println(F("Firmware OK"));

    // cache the mac address on startup
    WiFi.macAddress(macAddress);
    Serial.print(F("MAC: "));
    printMacAddress(macAddress);
}
void setupSRAMCache() {
    // 8k on board cache
    for (int i = 0; i < OnBoardSRAMCacheSize; ++i) {
        OnBoardSRAMCache[i].word = 0;
    }
}
// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    Serial.println(F("i960Sx chipset bringup"));
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::SPI_BUS_EN,
              i960Pinout::DISPLAY_EN,
              i960Pinout::SD_EN);
	digitalWrite(i960Pinout::SPI_BUS_EN, HIGH);
    digitalWrite(i960Pinout::SD_EN, HIGH);
    digitalWrite(i960Pinout::DISPLAY_EN, HIGH);
    pinMode(static_cast<int>(i960Pinout::Led), OUTPUT);
    t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
    SPI.begin();
	PinAsserter<i960Pinout::Reset960> holdi960InReset;
	setupIOExpanders();
	setupCPUInterface();
	setupBusStateMachine();
	setupWifi();
	setupSRAMCache();
	tft.begin();
	tft.fillScreen(ILI9341_BLACK);
	tft.setCursor(0,0);
	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(3);
	tft.println("i960Sx!");
	delay(1000);
	// we want to jump into the code as soon as possible after this point
	Serial.println(F("i960Sx chipset brought up fully!"));
}
void loop() {
	fsm.run_machine();
    t.update();
}
