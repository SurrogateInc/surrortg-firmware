#ifndef _SURRORTG_SRTG_H
#define _SURRORTG_SRTG_H

#if defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
#include <WiFiNINA.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

/**
 * An enum representing unsigned 8-bit command identifiers.
 * These identifiers are used to identify the command for which a certain
 * value is received by the server.
 */
enum class CommandId : uint8_t {
    THROTTLE = 1,
    THROTTLE_CAL = 2,
    STEER = 3,
    STEER_CAL = 4,
    CUSTOM_1 = 5,
    CUSTOM_1_CAL = 6,
    CUSTOM_2 = 7,
    CUSTOM_2_CAL = 8,
    CUSTOM_3 = 9,
    CUSTOM_3_CAL = 10,
    CUSTOM_4 = 11,
    CUSTOM_4_CAL = 12,
    /* Generic system commands */
    BATTERY_STATUS = 100,
    PING = 200,
    STOP = 0xFF
};

/**
 * A struct representing a command packet.
 * The data received by the server is packed into these packets.
 * The struct includes a CommandId and an 8-bit value for that command.
 */
struct CommandPacket {
    CommandId cmd;
    uint8_t value;
};

/**
 * A struct representing a callback.
 * The callback function is executed when a command is received by the server.
 * The CommandId is used to match the callback function with the right command
 * type.
 */
struct CommandCallback {
    CommandCallback(bool (*f)(uint8_t), CommandId c) : function(f), cmd(c) {}
    bool (*function)(uint8_t);
    CommandId cmd;
};

/**
 * A representation of a callback function.
 */
using CallbackFn = void (*)(void);

/**
 * A struct holding the necessary information about the callbacks.
 * command_cbs holds all the CommanCallbacks
 * command_cbs_len indicates the length of the command_cbs
 * reset_cb is called after no commands have been received for a specific time
 * continuous_cb is called on every loop iteration as long as some commands
 * are being received by the server
 */
struct SdkCallbacks {
    CommandCallback* command_cbs;
    size_t command_cbs_len;
    CallbackFn reset_cb;
    CallbackFn continuous_cb;
};

/**
 * A class for handling SurroRTG Arduino SDK.
 * Sets up a TCP server for receiving command messages from clients.
 * Supports multiple clients simultaneously and maintains the TCP connection
 * to the clients until they close the connection.
 */
class Srtg {
  public:
    /**
     * A constructor for Srtg.
     */
    Srtg() : server_(SRTG_SERVER_PORT){};

    /**
     * A function for setting up the SurroRTG Arduino SDK.
     * Should be called once in the setup phase of the main code.
     * Sets up the WiFi connection, OTA-update server and TCP command server.
     * @param ssid the WiFi network ssid
     * @param password the WiFi network password
     * @param ota_password the OverTheAir-update server password for securing
     *  the OTA-updates. Can be anything the user decides.
     * @param callbacks a SdkCallbacks pointer that points to a struct
     *  including the information about callbacks
     */
    void setup_sdk(const char* ssid,
                   const char* password,
                   const char* ota_password,
                   SdkCallbacks* callbacks);

    /**
     * A function for processing the SurroRTG Arduino SDK.
     * Should be called on every iteration of the loop phase of the main code.
     * Checks for incoming OTA-updates and messages and processes them.
     */
    void process_sdk();

    /**
     * A function for sending a response message back to the client.
     * The response is sent to the client that sent the most recently processed
     * command message. The response consists of a command id and a value.
     * @param cmd_id the command identifier as a CommandId
     * @param val the value to send as an unsigned char
     */
    void send_response(CommandId cmd_id, uint8_t val);

  private:
    void setup_wifi();
    void setup_ota();
    void process_server();
    bool process_client(int client_id);
    bool process_cmd(const CommandPacket* cmd);
    void check_client_connections();

    static constexpr unsigned long CONTROLS_DISABLE_DELAY_MS = 5000;
    static constexpr unsigned long WIFI_STATUS_INTERVAL_MS = 10000;
    static constexpr uint16_t SRTG_SERVER_PORT = 31338;
    static constexpr int MAX_CLIENTS = 3;
    const char* ssid_;
    const char* password_;
    const char* ota_password_;
    struct SdkCallbacks* callbacks_;
    class WiFiServer server_;
    class WiFiClient connected_clients_[MAX_CLIENTS];
    bool clients_alive_[MAX_CLIENTS] = {false};
    unsigned long last_network_action_ = 0;
    bool controls_enabled_ = false;
    int current_client_ = -1;
    uint16_t wifi_reboot_count_;
};
extern Srtg SRTG;

#endif  // _SURRORTG_SRTG_H