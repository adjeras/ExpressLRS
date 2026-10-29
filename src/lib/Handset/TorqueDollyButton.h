#pragma once
#include <stdint.h>

// One producer (power loop), atomic snapshots consumed by the handset task.
namespace TorqueDollyButton {
struct Snapshot {
    bool initialized = false;
    bool pressed = false;
    bool eligible = false; // A fresh press after the startup release.
    bool pending = false;  // Latched until power-off; never cleared on release.
    uint32_t sequence = 0;
    uint32_t pressedAt = 0;
    uint32_t duration = 0; // Frozen at the debounced release edge.
    uint32_t blinkStartedAt = 0;
};

class Input {
public:
    Snapshot snapshot;
    void update(bool raw, uint32_t now, uint32_t debounce, uint32_t shutdown) {
        if (!snapshot.initialized) {
            snapshot.initialized = true;
            snapshot.pressed = raw;
            snapshot.pressedAt = now;
            raw_ = raw;
            changedAt_ = now;
            // Even a released startup input must settle before arming.
            return;
        }
        if (raw != raw_) { raw_ = raw; changedAt_ = now; }
        if (!snapshot.pressed && !raw_ &&
            uint32_t(now - changedAt_) >= debounce) armed_ = true;
        if (raw_ != snapshot.pressed && uint32_t(now - changedAt_) >= debounce) {
            if (raw_) {
                snapshot.pressed = true;
                snapshot.eligible = armed_;
                snapshot.pressedAt = now;
                snapshot.duration = 0;
                ++snapshot.sequence;
            } else {
                snapshot.duration = uint32_t(now - snapshot.pressedAt);
                if (snapshot.eligible && snapshot.duration >= shutdown)
                    setPending(now);
                snapshot.pressed = false;
                ++snapshot.sequence;
                armed_ = true;
            }
        }
        if (snapshot.pressed && snapshot.eligible &&
            uint32_t(now - snapshot.pressedAt) >= shutdown) setPending(now);
    }
private:
    bool raw_ = false;
    bool armed_ = false;
    uint32_t changedAt_ = 0;
    void setPending(uint32_t now) {
        if (!snapshot.pending) snapshot.blinkStartedAt = now;
        snapshot.pending = true;
    }
};

enum class State { Locked, Ready, ControlledStop };
enum class Action { None, Lock, Unlock };
class Gesture {
public:
    void cancel() { eligible_ = false; }
    Action update(const Snapshot &s, State state, bool canUnlock,
                  uint32_t unlockMin, uint32_t shutdown) {
        if (!s.initialized) return Action::None;
        if (s.pending) eligible_ = false;
        if (!initialized_) {
            initialized_ = true;
            seen_ = s.sequence;
            // Never interpret an already-held press on consumer startup.
            return Action::None;
        }
        if (s.sequence == seen_) return Action::None;
        const bool consecutive = uint32_t(s.sequence - seen_) == 1;
        seen_ = s.sequence;
        if (!consecutive && state == State::Ready) {
            eligible_ = false;
            return Action::Lock; // A missed press must not leave motion enabled.
        }
        if (s.pressed) {
            eligible_ = consecutive && s.eligible && !s.pending && state == State::Locked;
            return state == State::Ready ? Action::Lock : Action::None;
        }
        const bool unlock = consecutive && eligible_ && s.eligible && !s.pending &&
            state == State::Locked && canUnlock &&
            s.duration >= unlockMin && s.duration < shutdown;
        eligible_ = false;
        return unlock ? Action::Unlock : Action::None;
    }
private:
    bool initialized_ = false;
    bool eligible_ = false;
    uint32_t seen_ = 0;
};
} // namespace TorqueDollyButton
