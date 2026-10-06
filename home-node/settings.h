#pragma once
#include <stdint.h>
// Brisbane uses UTC+10; weekdays are Monday through Friday.
constexpr const char* LOCAL_TZ = "AEST-10";
constexpr int WORK_END_MINUTE = 18 * 60;
// Count continuous inactivity after work ends, without an additional travel delay.
constexpr uint32_t IDLE_SECONDS = 30 * 60;
// Home remote-task duration, measured from the event timestamp. Local tasks use 1800000 ms in control.h.
constexpr uint32_t SESSION_SECONDS = 30 * 60;
// Maximum event age on receipt and maximum resend window at the work node.
constexpr uint32_t EVENT_MAX_AGE_SECONDS = 120;
// Work-node sampling interval. IDLE_SECONDS and WORK_SAMPLE_MS are not used by the home control loop.
constexpr uint32_t WORK_SAMPLE_MS = 100;
// IDLE_SECONDS may be shortened for testing; restore it to 1800 seconds afterwards.
// Use the real clock for dates, time zones and message times; do not simulate weekdays.
static_assert(IDLE_SECONDS >= 1 && IDLE_SECONDS <= 86400, "Check idle time");
static_assert(WORK_END_MINUTE >= 0 && WORK_END_MINUTE < 1440, "Check work time");
static_assert(SESSION_SECONDS >= 1 && SESSION_SECONDS <= 86400, "Check session time");
