#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace sa {
inline bool weekday(int day) { return day >= 1 && day <= 5; }
inline bool workWindow(int day, int minute, int endMinute) {
  return weekday(day) && minute >= endMinute && minute < 1440;
}
inline bool fresh(uint32_t event, uint32_t now, uint32_t maxAge = 120) {
  return event <= now && now - event <= maxAge;
}
// Use desk idle events; legacy parking messages are no longer accepted.
// Accept work-desk and, only with a test output pointer, work-test. This parser does not check age, mode or weekday.
inline bool parseEvent(const char* text, size_t length, uint32_t& epoch, bool* test = nullptr) {
  const char prefix[] = "v2|work-desk|";
  const size_t n = sizeof(prefix) - 1;
  if (length <= n || length > n + 10) return false;
  bool isTest = memcmp(text, "v2|work-test|", n) == 0;
  if (memcmp(text, prefix, n) && !isTest) return false;
  if (isTest && !test) return false;
  if (test) *test = isTest;
  uint64_t value = 0;
  for (size_t i = n; i < length; ++i) {
    if (text[i] < '0' || text[i] > '9') return false;
    value = value * 10 + (text[i] - '0');
  }
  if (value < 1700000000ULL || value > UINT32_MAX) return false;
  epoch = (uint32_t)value; return true;
}
class DeskIdleDetector {
 public:
  // Generate one event per date; firmware persists the date in nonvolatile storage.
  // line 34-36 return current idle time
  uint32_t idleElapsedMs(uint32_t now) const {
    return tracking ? now - quietAt : 0;
  }
  
  uint32_t firedDay = 0;
  bool sample(bool motion, bool enabled, uint32_t day, uint32_t now, uint32_t idleMs) {
    // Normal events only: reset the quiet interval when disabled, on a new date, or after a sample gap greater than one second. Motion also resets it.
    if (!enabled) { tracking = false; return false; }
    if (!tracking || currentDay != day || now - lastSample > 1000) {
      tracking = true; currentDay = day; quietAt = now;
    }
    lastSample = now;
    if (motion) quietAt = now;
    if (firedDay == day || now - quietAt < idleMs) return false;
    firedDay = day; return true;
  }
 private:
  bool tracking = false;
  uint32_t currentDay = 0, quietAt = 0, lastSample = 0;
};
}
