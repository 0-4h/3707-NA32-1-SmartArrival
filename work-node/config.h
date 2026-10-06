#pragma once
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#include "settings.h"
// Connect the desk PIR motion sensor digital output to GPIO27.
constexpr int PIR_PIN = 27;

// Test button and continuous-motion switch use pull-ups; ground activates them.
constexpr int TEST_LEAVE_PIN = 25, TEST_MOTION_PIN = 26;