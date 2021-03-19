#ifndef _SURRORTG_RRC_BATTERY_MANAGER_H
#define _SURRORTG_RRC_BATTERY_MANAGER_H

#include <Arduino.h>

struct BatteryStatus {
    uint8_t battery_charge;
    uint16_t charging;
    uint16_t status_word;
};

void setup_rrc_battery();
void update_battery_usage(BatteryStatus*);
void print_battery_status();

#endif  // _SURRORTG_RRC_BATTERY_MANAGER_H
