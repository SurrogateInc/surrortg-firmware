/*
    SurroRTG TCP test game that works with the SurroRTG SDK tcp_bot_dummy_game.
    The bots respond to inputs sent from the SurroRTG Python controller.

    This code works with ESP32 and ESP8266 based microcontrollers and
    Arduino MKR1000.

    For instructions, see https://docs.surrogate.tv/udp_bot_games.html
 */

#include <Arduino.h>
#include <surrortg.h>
#include "wifi_secrets.h"

namespace {

/* Sends back the received throttle value */
bool handle_throttle(uint8_t value_received) {
    Serial.print("Throttle cmd from client: ");
    Serial.println(value_received);
    SRTG.send_response(CommandId::THROTTLE, value_received);
    return true;
}

/* Sends back the received steering value */
bool handle_steering(uint8_t value_received) {
    Serial.print("Steering cmd from client: ");
    Serial.println(value_received);
    SRTG.send_response(CommandId::STEER, value_received);
    return true;
}

/* Sends back the received custom command value */
bool handle_custom_action(uint8_t value_received) {
    Serial.print("Action cmd from client: ");
    Serial.println(value_received);
    SRTG.send_response(CommandId::CUSTOM_1, value_received);
    if (value_received == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
    return true;
}

/* Map input handler functions to commands */
CommandCallback cmd_cbs[] = {
        CommandCallback(&handle_throttle, CommandId::THROTTLE),
        CommandCallback(&handle_steering, CommandId::STEER),
        CommandCallback(&handle_custom_action, CommandId::CUSTOM_1)};
SdkCallbacks cbs = {cmd_cbs, 3, nullptr, nullptr};

}  // End anonymous namespace

/* Setup WiFi and OTA, open TCP server */
void setup() {
    SRTG.setup_sdk(ssid, password, ota_password, &cbs);
    pinMode(LED_BUILTIN, OUTPUT);

    // Blink LED twice to inform successful boot
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
}

/* Process incoming messages and OTA-updates */
void loop() {
    SRTG.process_sdk();
}
