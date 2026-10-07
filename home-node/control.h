#pragma once
#include <stdint.h>
#include <math.h>
#include "schedule.h"

namespace sa {
inline uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }
// IDLE: no automatic task. PREPARE: wait for arrival. ARRIVAL: active task with AC off. COMFORT: active task with AC on after tick.
enum class State { IDLE, PREPARE, ARRIVAL, COMFORT };
enum class Mode { REMOTE, LOCAL };
enum class Command { DEPART, SYSTEM_OFF, RESET, LIGHT_ON, LIGHT_OFF, AC_ON, AC_OFF, AUTO, ETA, MODE_LOCAL, MODE_REMOTE };
inline const char* name(State state) {
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::PREPARE: return "PREPARE";
    case State::ARRIVAL: return "ARRIVAL";
    default: return "COMFORT";
  }
}
class Controller {
 public:
  State state = State::IDLE;
  Mode mode = Mode::REMOTE;
  const char* modeName() const { return mode == Mode::LOCAL ? "LOCAL" : "REMOTE"; }
  void restoreMode(bool local) { mode = local ? Mode::LOCAL : Mode::REMOTE; }
  bool light = false, ac = false, present = false;
  bool tempOK = false, distanceOK = false;
  float temperature = NAN, humidity = NAN, distance = NAN;
  const char* reason = "BOOT";
  uint32_t etaMs = 60000;
  void begin(uint32_t now) { bootAt = now; }
  // Both modes require valid, recent temperature, humidity and distance samples for automatic output changes.
  bool healthy(uint32_t now) const {
    return tempOK && distanceOK && elapsed(now, tempAt) <= 6000 && elapsed(now, rangeAt) <= 5000;
  }
  const char* sensorError(uint32_t now) const {
    if (badTemp || (seenTemp && elapsed(now, tempAt) > 6000)) return "HOME_DHT_ERROR";
    if (badRange || (seenRange && elapsed(now, rangeAt) > 5000)) return "HOME_RANGE_ERROR";
    if ((!seenTemp || !seenRange) && elapsed(now, bootAt) > 8000) return "HOME_SENSOR_MISSING";
    return nullptr;
  }
  void tempSample(float t, float h, uint32_t now) {
    temperature = t; humidity = h; tempAt = now; seenTemp = true;
    tempOK = isfinite(t) && isfinite(h) && t >= -40 && t <= 80 && h >= 0 && h <= 100;
    badTemp = !tempOK;
  }
  void distanceSample(float cm, uint32_t now) {
    distance = cm; rangeAt = now; seenRange = true;
    distanceOK = isfinite(cm) && cm >= 2 && cm <= 400; badRange = !distanceOK;
    if (!distanceOK) { nearCount = farCount = 0; return; }
    // Three consecutive samples below 80 cm set present. Three above 100 cm clear it. The 80-100 cm band holds the last presence state.
    if (cm < 80) {
      farCount = 0;
      if (nearCount < 3) ++nearCount;
      if (nearCount == 3) present = true;
    } else if (cm > 100) {
      nearCount = 0;
      if (farCount < 3) ++farCount;
      if (farCount == 3) present = false;
    } else { nearCount = farCount = 0; }
  }
  // Accept a remote task without another travel delay. Outputs are updated on the next healthy tick; this function does not set them.
  bool startWork(uint32_t now, uint32_t ageMs, uint32_t durationMs) {
    // A current remote task rejects new events without extending its duration. A local task can be replaced by a remote task.
    if (mode != Mode::REMOTE || scheduled || ageMs >= durationMs) return false;
    scheduled = true; receivedAt = now; eventAge = ageMs; runTime = durationMs;
    state = State::COMFORT; reason = "WORK_IDLE"; clearManual();
    return true;
  }
  bool command(Command command, uint32_t now, uint32_t value = 0) {
    // A mode change ends the task and clears the local block. A vehicle already present can start a new local task on the next healthy tick.
    if (command == Command::MODE_LOCAL || command == Command::MODE_REMOTE) {
      Mode next = command == Command::MODE_LOCAL ? Mode::LOCAL : Mode::REMOTE;
      if (next == mode) return true;
      end("MODE_CHANGED"); mode = next; localBlocked = false; return true;
    }
    if (command == Command::SYSTEM_OFF || command == Command::RESET) {
      end(command == Command::RESET ? "RESET" : "STOP"); return true;
    }
    // Allow the user to turn outputs off even when a sensor fails.
    if (command == Command::LIGHT_OFF) { lightMode = 0; lightAt = now; light = false; return true; }
    if (command == Command::AC_OFF) { acMode = 0; acAt = now; ac = false; return true; }
    if (!healthy(now)) return false;
    // ETA only sets the PREPARE arrival timeout, with 30 extra seconds. It does not delay a work event or start outputs.
    if (command == Command::ETA) {
      if (value < 5 || value > 7200 || scheduled) return false;
      etaMs = value * 1000; return true;
    }
    // In LOCAL, DEPART clears the local block. In REMOTE, it arms PREPARE only from IDLE with no vehicle present.
    if (command == Command::DEPART) {
      if (state == State::PREPARE) return true;
      if (mode == Mode::LOCAL) { localBlocked = false; return true; }
      if (state != State::IDLE || present) return false;
      state = State::PREPARE; departAt = now; reason = "ARMED";
      light = ac = false; clearManual(); return true;
    }
    // AUTO clears both manual overrides. It does not change LOCAL/REMOTE mode or start a task.
    if (command == Command::AUTO) { clearManual(); return true; }
    if (command == Command::LIGHT_ON) { lightMode = 1; lightAt = now; light = true; return true; }
    if (command == Command::AC_ON) {
      if (!active() || (!scheduled && !present)) return false;
      acMode = 1; acAt = now; ac = true; return true;
    }
    return false;
  }
  void tick(uint32_t now) {
    // Skip automatic updates, manual-override expiry and timeout checks while sensors are unhealthy. Time still passes; commands can still change state or turn outputs off.
    if (!healthy(now)) return;
    // Manual output overrides last 60 seconds. Safety and task-end conditions can end an override sooner.
    if (lightMode != -1 && elapsed(now, lightAt) >= 60000) lightMode = -1;
    if (acMode != -1 && elapsed(now, acAt) >= 60000) acMode = -1;
    // A remote task does not require a vehicle. Its duration includes message age; there is no season check.
    if (scheduled) {
      uint64_t age = (uint64_t)eventAge + elapsed(now, receivedAt);
      if (age >= runTime) { end("SESSION_TIMEOUT"); return; }
      light = true;
      // AC turns on above 26 C and off below 22 C. From 22 through 26 C it holds its state. This can cycle more than once in one task.
      if (temperature > 26) ac = true;
      else if (temperature < 22) ac = false;
      manualOutputs();
      state = ac ? State::COMFORT : State::ARRIVAL;
      return;
    }
    {
      // LOCAL and REMOTE both use local arrival detection when no remote task is active.
      if (!present) localBlocked = false;
      if (state == State::IDLE && present && !localBlocked) {
        state = State::ARRIVAL; arrivedAt = now; missing = false; reason = "LOCAL_ARRIVAL";
      }
    }
    if (state == State::PREPARE) {
      if (elapsed(now, departAt) >= etaMs + 30000) { end("ARRIVAL_TIMEOUT"); return; }
      if (present) { state = State::ARRIVAL; arrivedAt = now; reason = "ARRIVED"; missing = false; }
    }
    // For a local task, absence turns AC off at once and ends the task after 10 seconds. The task also ends after 30 minutes.
    if (active()) {
      if (!present) {
        if (!missing) { missing = true; missingAt = now; }
        ac = false;
        if (elapsed(now, missingAt) >= 10000) { end("VEHICLE_LEFT"); return; }
      } else missing = false;
      if (elapsed(now, arrivedAt) >= 1800000) { end("SESSION_TIMEOUT"); return; }
    }
    light = active();
    if (!active() || !present) ac = false;
    else {
      if (temperature > 26) ac = true;
      else if (temperature < 22) ac = false;
      if (acMode != -1) ac = acMode == 1;
    }
    if (active()) state = ac ? State::COMFORT : State::ARRIVAL;
    if (lightMode != -1) light = lightMode == 1;
  }
  // Use a fixed 200 ms interval; this version has no adaptive distance sampling.
  uint32_t distancePeriod(uint32_t /*now*/) const {
    return 200;

  }
 private:
  uint32_t bootAt = 0, tempAt = 0, rangeAt = 0, departAt = 0, arrivedAt = 0, missingAt = 0;
  uint32_t lightAt = 0, acAt = 0, receivedAt = 0, eventAge = 0, runTime = 0;
  bool seenTemp = false, seenRange = false, badTemp = false, badRange = false;
  bool missing = false, scheduled = false, localBlocked = false;
  uint8_t nearCount = 0, farCount = 0;
  int8_t lightMode = -1, acMode = -1;
  bool active() const { return state == State::ARRIVAL || state == State::COMFORT; }
  void clearManual() { lightMode = acMode = -1; }
  void manualOutputs() {
    if (lightMode != -1) light = lightMode == 1;
    if (acMode != -1 && active()) ac = acMode == 1;
  }
  // Stop both outputs and block another local arrival until absence or an applicable command clears the block. New remote tasks can still be accepted.
  void end(const char* why) {
    state = State::IDLE; light = ac = false; scheduled = missing = false; localBlocked = true;
    clearManual(); reason = why;
  }
};
}
