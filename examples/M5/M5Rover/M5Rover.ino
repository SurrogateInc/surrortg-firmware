/*
    SurroRTG M5 rover example that works with the SurroRTG SDK m5_rover game.
    The M5 rovers receive inputs from the SurroRTG Python controller, and can
    be controller from Surrogate.tv

    This code works with the M5StickC + RoverC and requires the M5StickC library
    to be installed through Arduino Library Manager.

    For instructions, see https://docs.surrogate.tv/udp_bot_games.html
 */

#include <M5StickC.h>
#include <surrortg.h>
#include "wifi_secrets.h"

namespace {

/* Motor definitions */
struct M5Motor {
    int8_t target_speed;
    int8_t current_speed;
    const uint8_t addr;
};
M5Motor M_FRONT_LEFT = {0, 0, 0x00};
M5Motor M_FRONT_RIGHT = {0, 0, 0x01};
M5Motor M_REAR_LEFT = {0, 0, 0x02};
M5Motor M_REAR_RIGHT = {0, 0, 0x03};
M5Motor* motors[4] = {&M_FRONT_LEFT, &M_FRONT_RIGHT, &M_REAR_LEFT,
                      &M_REAR_RIGHT};

/* Throttle for forward and back movement */
uint8_t requested_throttle;
int8_t active_throttle;

/* Steering the bot around it's axle */
uint8_t requested_steering;
int8_t stationary_turn;
int8_t curve_turn;

/* Throttle for sideways movement */
uint8_t requested_sideways_throttle;
int8_t active_sideways_throttle;

/* Throttle range is 0 - 127 */
/* Commands from the Python controller are in the range 0 - 200, where 100 is
 * middle */
constexpr uint8_t SPEED_MAX = 127;
constexpr int8_t SPEED_OFF = 0;
constexpr uint8_t CMD_MIN = 0;
constexpr uint8_t CMD_MID = 100;
constexpr uint8_t CMD_MAX = 200;
constexpr uint8_t MAX_STATIONARY_TURN = 40;
constexpr float CURVE_TURN_MULTIPLIER = 0.8;
constexpr unsigned long COMMAND_UPDATE_INTERVAL_MS = 10;
constexpr unsigned long SCREEN_UPDATE_INTERVAL_MS = 5000;
constexpr uint8_t RAMP_STEP = SPEED_MAX / 20;
constexpr uint8_t ROVER_ADDRESS = 0x38;
unsigned long last_command_update = 0;
unsigned long last_screen_update = 0;

/* Write speed values to the motor controller on the M5 Rover using I2C */
void i2c_write_speeds(uint8_t reg, int8_t* speeds, size_t size) {
    Wire.beginTransmission(ROVER_ADDRESS);
    Wire.write(reg);
    for (size_t i = 0; i < size; ++i) {
        Wire.write(speeds[i]);
    }
    Wire.endTransmission();
}

/* Drives all motors with the speeds stored in the motor definitions.
The speed of acceleration is reduced with ramp to prevent voltage drop
in the battery that would cause M5StickC to shutdown */
void drive_motors() {
    last_command_update = millis();
    int8_t speed_buf[4] = {0};
    for (uint8_t i = 0; i < 4; i++) {
        // Reduce speed with ramp
        if (motors[i]->current_speed > motors[i]->target_speed) {
            motors[i]->current_speed =
                    (motors[i]->current_speed <
                     motors[i]->target_speed + RAMP_STEP)
                            ? motors[i]->target_speed
                            : motors[i]->current_speed - RAMP_STEP;
        }
        // Increase speed with ramp
        else if (motors[i]->current_speed < motors[i]->target_speed) {
            motors[i]->current_speed =
                    (motors[i]->current_speed >
                     motors[i]->target_speed - RAMP_STEP)
                            ? motors[i]->target_speed
                            : motors[i]->current_speed + RAMP_STEP;
        }
        speed_buf[i] = motors[i]->current_speed;
    }
    i2c_write_speeds(0x00, speed_buf, 4);
}

void control_motors() {
    /* No forward, backward or sideways movement */
    if (active_throttle == SPEED_OFF && active_sideways_throttle == SPEED_OFF) {
        M_FRONT_LEFT.target_speed = stationary_turn;
        M_FRONT_RIGHT.target_speed = -stationary_turn;
        M_REAR_RIGHT.target_speed = -stationary_turn;
        M_REAR_LEFT.target_speed = stationary_turn;
    }
    /* Only sideways movement */
    else if (active_throttle == SPEED_OFF &&
             active_sideways_throttle != SPEED_OFF) {
        M_FRONT_LEFT.target_speed = active_sideways_throttle;
        M_FRONT_RIGHT.target_speed = -active_sideways_throttle;
        M_REAR_RIGHT.target_speed = active_sideways_throttle;
        M_REAR_LEFT.target_speed = -active_sideways_throttle;
    }
    /* Diagonal movement */
    else if (active_throttle != SPEED_OFF &&
             active_sideways_throttle != SPEED_OFF) {
        // Rear-right or front-left direction
        if ((active_sideways_throttle > SPEED_OFF &&
             active_throttle < SPEED_OFF) ||
            (active_sideways_throttle < SPEED_OFF &&
             active_throttle > SPEED_OFF)) {
            M_FRONT_LEFT.target_speed = SPEED_OFF;
            M_FRONT_RIGHT.target_speed = active_throttle;
            M_REAR_RIGHT.target_speed = SPEED_OFF;
            M_REAR_LEFT.target_speed = active_throttle;
        }
        // Rear-left or front-right direction
        else {
            M_FRONT_LEFT.target_speed = active_throttle;
            M_FRONT_RIGHT.target_speed = SPEED_OFF;
            M_REAR_RIGHT.target_speed = active_throttle;
            M_REAR_LEFT.target_speed = SPEED_OFF;
        }
    }
    /* Forward or backward movement */
    else {
        M_FRONT_LEFT.target_speed = active_throttle;
        M_FRONT_RIGHT.target_speed = active_throttle;
        M_REAR_RIGHT.target_speed = active_throttle;
        M_REAR_LEFT.target_speed = active_throttle;

        if (stationary_turn < SPEED_OFF) {
            M_FRONT_LEFT.target_speed += curve_turn;
            M_REAR_LEFT.target_speed += curve_turn;
        } else if (stationary_turn > SPEED_OFF) {
            M_FRONT_RIGHT.target_speed -= curve_turn;
            M_REAR_RIGHT.target_speed -= curve_turn;
        }
    }

    drive_motors();
}

/* Update the throttle value based on input */
bool update_throttle(uint8_t data) {
    requested_throttle = constrain(data, CMD_MIN, CMD_MAX);

    // Map the input from command range 0 - 200 to throttle range (-127) - 127
    active_throttle =
            map(requested_throttle, CMD_MIN, CMD_MAX, -SPEED_MAX, SPEED_MAX);
    control_motors();
    return true;
}

/* Update the steering value based on input */
bool update_steering(uint8_t data) {
    requested_steering = constrain(data, CMD_MIN, CMD_MAX);

    // Map the input from command range 0 - 200 to turning ranges
    stationary_turn = map(requested_steering, CMD_MIN, CMD_MAX,
                          -MAX_STATIONARY_TURN, MAX_STATIONARY_TURN);
    curve_turn = map(requested_steering, CMD_MIN, CMD_MAX,
                     -active_throttle * CURVE_TURN_MULTIPLIER,
                     active_throttle * CURVE_TURN_MULTIPLIER);
    control_motors();
    return true;
}

/* Update the sideways throttle value based on input */
bool update_sideways_throttle(uint8_t data) {
    requested_sideways_throttle = constrain(data, CMD_MIN, CMD_MAX);

    // Map the input from command range 0 - 200 to throttle range (-127) - 127
    active_sideways_throttle = map(requested_sideways_throttle, CMD_MIN,
                                   CMD_MAX, -SPEED_MAX, SPEED_MAX);
    control_motors();
    return true;
}

/* Stop the robot and update throttle values to OFF */
void stop_robot() {
    active_throttle = SPEED_OFF;
    stationary_turn = SPEED_OFF;
    curve_turn = SPEED_OFF;
    active_sideways_throttle = SPEED_OFF;
    control_motors();
}

/* Map input handler functions to commands */
CommandCallback cmd_cbs[] = {
        CommandCallback(&update_throttle, CommandId::THROTTLE),
        CommandCallback(&update_steering, CommandId::STEER),
        CommandCallback(&update_sideways_throttle, CommandId::CUSTOM_1)};
SdkCallbacks cbs = {cmd_cbs, 3, &stop_robot, nullptr};

}  // End anonymous namespace

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

/*
 * Setup WiFi and OTA, open TCP server.
 * Initialize M5 Rover and light up screen
 */
void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    tftSprite.createSprite(160, 80);
    M5.Axp.EnableCoulombcounter();
    M5.Axp.SetLDO2(false);
    SRTG.setup_sdk(ssid, password, ota_password, &cbs);
    Wire.begin(0, 26, 100);  // sda 0, scl 26, frequency 100
    stop_robot();
}

/*
 * Process incoming messages and OTA-updates
 */
void loop() {
    SRTG.process_sdk();

    unsigned long time_now = millis();

    // Drive motors only once in COMMAND_UPDATE_INTERVAL_MS milliseconds
    if (time_now - last_command_update > COMMAND_UPDATE_INTERVAL_MS) {
        drive_motors();
    }

    // Show battery information on LCD screen only when charging
    if (time_now - last_screen_update > SCREEN_UPDATE_INTERVAL_MS) {
        if (M5.Axp.GetVBusVoltage() > 4.0) {
            M5.Axp.SetLDO2(true);
            float current = M5.Axp.GetVBusCurrent() - M5.Axp.GetBatCurrent();
            Serial.printf("Current usage I: %.3fm\r\n", current);
            tftSprite.fillSprite(BLACK);
            tftSprite.setCursor(0, 0, 1);
            tftSprite.printf("AXP Temp: %.1fC \r\n", M5.Axp.GetTempInAXP192());
            tftSprite.setCursor(0, 10);
            tftSprite.printf("Bat:\r\n  V: %.3fv  I: %.3fma\r\n",
                             M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
            tftSprite.setCursor(0, 30);
            tftSprite.printf("USB:\r\n  V: %.3fv  I: %.3fma\r\n",
                             M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());
            tftSprite.setCursor(0, 50);
            tftSprite.printf("5V-In:\r\n  V: %.3fw  I: %.3fma\r\n",
                             M5.Axp.GetVinVoltage(), M5.Axp.GetVinCurrent());
            tftSprite.setCursor(0, 70);
            tftSprite.printf("Bat power %.3fmw", M5.Axp.GetBatPower());
            tftSprite.pushSprite(0, 0);
        }
        // Turn off LCD screen's power if not charging to save battery
        else {
            M5.Axp.SetLDO2(false);
        }
        last_screen_update = millis();
    }
}