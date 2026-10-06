#pragma once
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#include "settings.h"
constexpr int DHT_PIN = 16, TRIG_PIN = 18, ECHO_PIN = 19;
constexpr int LIGHT_RELAY_PIN = 25, AC_PIN = 26, MODE_BUTTON_PIN = 23;
constexpr int MODE_LED_PIN = 27;
// Enable only for serial fault tests: f injects a DHT error; n restores sensor readings.
constexpr bool DEMO_FAULT_COMMANDS = false;
