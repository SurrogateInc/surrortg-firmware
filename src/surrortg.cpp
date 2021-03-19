#include "surrortg.h"

#if defined(ARDUINO_SAMD_MKR1000)
#include <FlashStorage.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <utility/rrc_battery_manager.h>
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#include <ArduinoOTA.h>
#include <FlashStorage.h>
#include <SPI.h>
#include <WiFiNINA.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <ArduinoOTA.h>
#include <WiFi.h>
#else
#error "This library only works with ESP32 and ESP8266 based boards and Arduino MKR1000 / MKR1010"
#endif

#if defined(ARDUINO_ARCH_SAMD)
FlashStorage(wifi_reboot_counter, uint16_t);
#endif

Srtg SRTG = Srtg();

void Srtg::setup_sdk(const char* ssid,
                     const char* password,
                     const char* ota_password,
                     SdkCallbacks* callbacks) {
    ssid_ = ssid;
    password_ = password;
    ota_password_ = ota_password;
    callbacks_ = callbacks;

    Serial.begin(115200);

#if defined(ARDUINO_ARCH_SAMD)
    wifi_reboot_count_ = wifi_reboot_counter.read();
#endif

    Serial.println("Connecting to WiFi...");
    setup_wifi();

    Serial.println("Setting up OTA...");
    setup_ota();

    Serial.print("Setting up SRTG server on port ");
    Serial.print(SRTG_SERVER_PORT);
    Serial.println("...");
    server_.begin();

#if defined(ARDUINO_SAMD_MKR1000)
    Serial.println("Setting up battery status manager...");
    setup_rrc_battery();
#endif

    Serial.println("Boot finished!");
    Serial.flush();
}

void Srtg::process_sdk() {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    ArduinoOTA.handle();
#elif defined(ARDUINO_SAMD_MKR1000)
    WiFiOTA.poll();
#else
    ArduinoOTA.poll();
#endif

    static unsigned long last_print_time = 0;

    if (millis() - last_print_time > WIFI_STATUS_INTERVAL_MS) {
        last_print_time = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Reconnecting WiFi...");
            setup_wifi();
        } else {
            IPAddress ip_addr = WiFi.localIP();
            Serial.print("WiFi OK, IP address: ");
            Serial.println(ip_addr);
        }
    }

    process_server();
}

void Srtg::send_response(CommandId cmd_id, uint8_t val) {
    static CommandPacket response = {cmd_id, val};
    if (current_client_ >= 0) {
        connected_clients_[current_client_].write((uint8_t*)&response, 2);
    }
}

void Srtg::setup_wifi() {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_, password_);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Retrying");
        delay(5000);
        ESP.restart();
    }
#else
    static uint8_t wifi_retries = 0;
    while (WiFi.begin(ssid_, password_) != WL_CONNECTED) {
        Serial.println("Connection Failed! Retrying");
        wifi_retries++;

        if (wifi_retries > 3) {
#if defined(ARDUINO_ARCH_SAMD)
            Serial.println("WiFi retry limit reached! Resetting system");
            wifi_reboot_counter.write(wifi_reboot_count_ + 1);
            delay(1000);
            NVIC_SystemReset();
#else
            Serial.println("WiFi retry limit reached!");
            Serial.println("No restart method configured for this board");
#endif
        }
        delay(5000);
    }
    wifi_retries = 0;
#endif

    IPAddress ip_addr = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(ip_addr);

    byte mac_addr[6];
    WiFi.macAddress(mac_addr);
    Serial.print("MAC: ");
    for (int i = 5; i >= 0; --i) {
        if (mac_addr[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(mac_addr[i], HEX);
        if (i > 0) {
            Serial.print(":");
        }
    }
    Serial.println("");
}

void Srtg::setup_ota() {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    ArduinoOTA.setPassword(ota_password_);

    ArduinoOTA.onStart([]() { Serial.println("Start"); });

    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.begin();
#elif defined(ARDUINO_SAMD_MKR1000)
    WiFiOTA.begin("Arduino", ota_password_, InternalStorage);
#else
    ArduinoOTA.begin(WiFi.localIP(), "Arduino", ota_password_, InternalStorage);
#endif
}

void Srtg::process_server() {
    check_client_connections();

    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (process_client(i)) {
            controls_enabled_ = true;
            last_network_action_ = millis();
        }
    }

    if (controls_enabled_) {
        /* If we haven't received commands for a while, disable all controls */
        if (millis() - last_network_action_ > CONTROLS_DISABLE_DELAY_MS) {
            controls_enabled_ = false;
            if (callbacks_->reset_cb) {
                callbacks_->reset_cb();
            }
        }

        else {
            if (callbacks_->continuous_cb) {
                callbacks_->continuous_cb();
            }
        }
    }
}

bool Srtg::process_client(int client_id) {
    static CommandPacket packet;

    if (connected_clients_[client_id].available() >= 2) {
        if (!connected_clients_[client_id].connected()) {
            Serial.print("Executing command from disconnected client ");
            Serial.print(client_id);
            Serial.println(" buffer");
        }

        packet.cmd = (CommandId)connected_clients_[client_id].read();
        packet.value = (uint8_t)connected_clients_[client_id].read();

        current_client_ = client_id;

        return process_cmd(&packet);
    }

    return false;
}

bool Srtg::process_cmd(const CommandPacket* cmd) {
    /* All implementation specific commands are handled here */
    for (CommandCallback* temp_cb = callbacks_->command_cbs;
         temp_cb < callbacks_->command_cbs + callbacks_->command_cbs_len;
         temp_cb++) {
        if (temp_cb && (cmd->cmd == temp_cb->cmd)) {
            if (*temp_cb->function) {
                return temp_cb->function(cmd->value);
            } else {
                Serial.print("No callback function specified for cmd ");
                Serial.println((uint8_t)temp_cb->cmd);
                return false;
            }
        }
    }

    /* Switch-case only has generic system commands */
    switch (cmd->cmd) {
    case CommandId::PING: {
        Serial.println("Ping packet received, sending response");
        send_response(CommandId::PING, 0);
        return false;
    }

    case CommandId::STOP: {
        if (callbacks_->reset_cb) {
            callbacks_->reset_cb();
            controls_enabled_ = false;
            return false;
        }
    }

#if defined(ARDUINO_SAMD_MKR1000)
    case CommandId::BATTERY_STATUS: {
        if (current_client_ >= 0) {
            BatteryStatus status;
            update_battery_usage(&status);
            connected_clients_[current_client_].write(
                    (char*)&status.battery_charge, 1);
            connected_clients_[current_client_].write((char*)&status.charging,
                                                      2);
            connected_clients_[current_client_].write(
                    (char*)&status.status_word, 2);
            connected_clients_[current_client_].write(
                    (char*)&wifi_reboot_count_, 2);
        } else {
            Serial.println(
                    "No current client set, cannot send battery status!");
        }
        return false;
    }
#endif

    default: {
        Serial.print("Invalid command type received: ");
        Serial.println((uint8_t)(cmd->cmd));
        return false;
    }
    }
}

void Srtg::check_client_connections() {
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (!connected_clients_[i].connected() && clients_alive_[i]) {
            Serial.print("Client ");
            Serial.print(i);
            Serial.println(" disconnected");
            clients_alive_[i] = false;
        }
    }

    WiFiClient new_client = server_.available();
    if (new_client) {
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (!connected_clients_[i].connected() && !clients_alive_[i] &&
                connected_clients_[i].available() < 2) {
                Serial.print("New connection accepted to slot ");
                Serial.println(i);
                connected_clients_[i] = new_client;
                clients_alive_[i] = true;
                return;
            }
        }
        Serial.print(
                "New connection refused, already connected to max amount of ");
        Serial.print(MAX_CLIENTS);
        Serial.println(" clients");
        new_client.stop();
    }
}
