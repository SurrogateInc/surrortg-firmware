/************************************************************************************

PackProbe

by Erik Speckman

For documentation and the latest version see:
http://powercartel.com/projects/packprobe/ Source Repository:
git@github.com:PowerCartel/PackProbe.git

Depends on SoftI2CMaster http://playground.arduino.cc/Main/SoftwareI2CLibrary

PackProbe allows you to obtain detailed history and health information on laptop
battery packs without access to the corresponding laptop.

This is is a proof of concept. It doesn't handle error conditions and spurious
values

PackProbe is based on:
  http://linuxehacking.blogspot.com/2014/03/recharging-and-reusing-acer-laptop.html
  and http://forum.arduino.cc/index.php?topic=62955.0

*************************************************************************************/

/************************************************************************************

Configure and then load the SoftI2CMaster library

The default is for data (SDA) on DPin6 and clock (SDC) on DPin8 on the Arduino
Yun.

For more details on configuring the library, see:
  http://playground.arduino.cc/Main/SoftwareI2CLibrary

For background on Arduino Port and Pin Mapping, see:
  http://www.arduino.cc/en/Reference/PortManipulation

To find the pin mappings for your board see:
  Atmega 168/328 MCU (Uno, Mini, Nano, Fio, Pro):
    http://arduino.cc/en/Hacking/PinMapping168

  Atmega 32u4 MCU (Yun, Leonardo, Micro):
    http://arduino.cc/en/Hacking/PinMapping32u4


*************************************************************************************/
#include "rrc_battery_manager.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
// standard I2C address for Smart Battery packs
constexpr uint8_t DEVICE_ADDRESS = 11;

/************************************************************************************

Output Selection

This sketch was made to run on the WiFi-enabled Arduino Yun.

The default programming and output connection of the Yun is over TCP/IP
rather than the serial connection used by most other Arduino boards.

The default output on the Yun is available through the Console library.

If you are using this sketch on board with a serial port, you'll need to
#include <Serial.h> instead and change every instance of "Console." in the
code to "Serial." In addition you need to comment/uncomment the necessary
code in the setup function before compiling and installing on your board.

We are looking for a clean way to simplify this configuration in the future,
or make it automatic. Suggestions welcome.

*************************************************************************************/

// Standard and common non-standard Smart Battery commands
constexpr uint8_t BATTERY_MODE = 0x03;
constexpr uint8_t TEMPERATURE = 0x08;
constexpr uint8_t VOLTAGE = 0x09;
constexpr uint8_t CURRENT = 0x0A;
constexpr uint8_t AVG_CURRENT = 0x0B;
constexpr uint8_t RELATIVE_SOC = 0x0D;
constexpr uint8_t ABSOLUTE_SOC = 0x0E;
constexpr uint8_t REMAINING_CAPACITY = 0x0F;
constexpr uint8_t FULL_CHARGE_CAPACITY = 0x10;
constexpr uint8_t RUN_TIME_TO_EMPTY = 0x11;
constexpr uint8_t AVG_TIME_TO_EMPTY = 0x12;
constexpr uint8_t TIME_TO_FULL = 0x13;
constexpr uint8_t CHARGING_CURRENT = 0x14;
constexpr uint8_t CHARGING_VOLTAGE = 0x15;
constexpr uint8_t BATTERY_STATUS = 0x16;
constexpr uint8_t CYCLE_COUNT = 0x17;
constexpr uint8_t DESIGN_CAPACITY = 0x18;
constexpr uint8_t DESIGN_VOLTAGE = 0x19;
constexpr uint8_t SPEC_INFO = 0x1A;
constexpr uint8_t MFG_DATE = 0x1B;
constexpr uint8_t SERIAL_NUM = 0x1C;
constexpr uint8_t MFG_NAME = 0x20;   // String
constexpr uint8_t DEV_NAME = 0x21;   // String
constexpr uint8_t CELL_CHEM = 0x22;  // String
constexpr uint8_t MFG_DATA = 0x23;   // String
// Indidual cell voltages don't work on Lenovo and Dell Packs
constexpr uint8_t CELL4_VOLTAGE = 0x3C;
constexpr uint8_t CELL3_VOLTAGE = 0x3D;
constexpr uint8_t CELL2_VOLTAGE = 0x3E;
constexpr uint8_t CELL1_VOLTAGE = 0x3F;
constexpr uint8_t STATE_OF_HEALTH = 0x4F;

namespace {

void scan() {
    for (uint8_t i = 0; i < 127; i++) {
        Serial.print("Address: 0x");
        Serial.print(i, HEX);
        Wire.beginTransmission(i);
        uint8_t ack = Wire.endTransmission();
        if (ack == 0) {
            Serial.println(": OK");
            Serial.flush();
        } else {
            Serial.println(": -");
            Serial.flush();
        }
    }
}

int fetch_word(uint8_t func) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(func);
    Wire.endTransmission();
    uint8_t num_bytes = Wire.requestFrom(DEVICE_ADDRESS, (uint8_t)2);
    if (num_bytes >= 2) {
        uint8_t b1 = Wire.read();
        uint8_t b2 = Wire.read();

        return (int)b1 | (((int)b2) << 8);
    } else {
        Serial.println("Error fetching word");
        return 0;
    }
}

uint8_t i2c_smbus_read_block(uint8_t command,
                             uint8_t* block_buffer,
                             uint8_t block_buffer_len) {
    uint8_t x, num_bytes;
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(command);
    Wire.endTransmission();
    // num of bytes; 1 byte will be index 0
    num_bytes = Wire.requestFrom(DEVICE_ADDRESS, (uint8_t)32);
    // room for null at the end
    num_bytes = constrain(num_bytes, 0, block_buffer_len - 2);
    for (x = 0; x < num_bytes - 1; x++) {
        // -1 because x=num_bytes-1 if x<y; last byte needs to be "nack"'d,
        // x<y-1
        block_buffer[x] = Wire.read();
    }
    block_buffer[x++] = Wire.read();  // this will nack the last byte and store
                                      // it in x's num_bytes-1 address.
    block_buffer[x] = 0;              // and null it at last_byte+1

    return num_bytes;
}

}  // End anonymous namespace

void setup_rrc_battery() {
    Wire.begin();
    Serial.println("I2C Inialized");
    scan();
}

void update_battery_usage(BatteryStatus* battery_state) {
    battery_state->battery_charge = fetch_word(RELATIVE_SOC);
    battery_state->charging = fetch_word(CHARGING_CURRENT);
    battery_state->status_word = fetch_word(BATTERY_STATUS);
    Serial.println(battery_state->status_word, BIN);
    Serial.println(battery_state->battery_charge);
}

void print_battery_status() {
    Serial.println("GENERIC INFORMATION");
    Serial.print("Cycle Count: ");
    Serial.println(fetch_word(CYCLE_COUNT));

    Serial.print("Battery Status (BIN): ");
    Serial.println(fetch_word(BATTERY_STATUS), BIN);

    Serial.print("Full/Remaining Charge Capacity: ");
    Serial.print(fetch_word(FULL_CHARGE_CAPACITY));
    Serial.print("/");
    Serial.println(fetch_word(REMAINING_CAPACITY));

    Serial.print("Relative Charge(%): ");
    Serial.println(fetch_word(RELATIVE_SOC));

    Serial.print("Absolute Charge(%): ");
    Serial.println(fetch_word(ABSOLUTE_SOC));
    Serial.println("");
    Serial.println("CHARGING INFORMATION:");
    Serial.print("Minutes remaining for full charge: ");
    Serial.println(fetch_word(TIME_TO_FULL));

    Serial.print("Charging Current: ");
    Serial.println(fetch_word(CHARGING_CURRENT));

    Serial.print("Charging Voltage: ");
    Serial.println(fetch_word(CHARGING_VOLTAGE));

    Serial.print("State of Health: ");
    Serial.println(fetch_word(STATE_OF_HEALTH));

    Serial.print("Battery Mode (BIN): ");
    Serial.println(fetch_word(BATTERY_MODE), BIN);
    Serial.println("");
    Serial.println("CURRENT USAGE:");
    Serial.print("Temp: ");
    unsigned int tempk = fetch_word(TEMPERATURE);
    Serial.println((float)tempk / 10.0 - 273.15);

    Serial.print("Current (mA): ");
    Serial.println(fetch_word(CURRENT));

    Serial.print("Average current (mA): ");
    Serial.println(fetch_word(AVG_CURRENT));

    Serial.print("Voltage: ");
    Serial.println((float)fetch_word(VOLTAGE) / 1000);

    Serial.println("");
}
