#pragma once
#include <stdint.h>

namespace sa {
// Debounce press and release for 40 milliseconds; holding generates only one event.
class ModeButton {
 public:
  bool sample(bool pressed, uint32_t now) {
    if (pressed != raw) { raw = pressed; changedAt = now; }
    if (raw == stable || uint32_t(now - changedAt) < 40) return false;
    stable = raw;
    return stable;
  }
 private:
  bool raw = false, stable = false;
  uint32_t changedAt = 0;
};
}
