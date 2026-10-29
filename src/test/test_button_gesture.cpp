#include <cassert>
#include <cstdint>

enum class State { Locked, Ready, ControlledStop };

struct Gesture {
    State state = State::Locked;
    bool startupReleaseRequired = true;
    bool stablePressed = false;
    bool unlockEligible = false;
    bool consumed = true;
    bool shutdownPending = false;
    uint32_t changedAt = 0;
    uint32_t pressedAt = 0;

    static constexpr uint32_t Debounce = 30;
    static constexpr uint32_t UnlockMin = 700;
    static constexpr uint32_t Shutdown = 3000;

    void sample(bool pressed, uint32_t now, bool calibrated = true, bool neutral = true) {
        static bool raw = false;
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            raw = pressed;
            stablePressed = pressed;
            changedAt = now;
            startupReleaseRequired = pressed;
            if (pressed) pressedAt = now;
            return;
        }
        if (pressed != raw) {
            raw = pressed;
            changedAt = now;
        }
        if (stablePressed != raw && now - changedAt >= Debounce) {
            stablePressed = raw;
            if (!stablePressed) {
                const uint32_t duration = now - pressedAt;
                const bool shutdown = shutdownPending || duration >= Shutdown;
                if (startupReleaseRequired) {
                    startupReleaseRequired = false;
                } else if (state == State::Locked && unlockEligible && !consumed &&
                           !shutdown && duration >= UnlockMin && duration < Shutdown &&
                           calibrated && neutral) {
                    state = State::Ready;
                }
                consumed = consumed || shutdown;
                unlockEligible = false;
                shutdownPending = false;
            } else {
                pressedAt = now;
                unlockEligible = !startupReleaseRequired && state == State::Locked;
                consumed = !unlockEligible;
                if (state == State::Ready) state = State::ControlledStop;
            }
        }
        if (stablePressed && !startupReleaseRequired && !shutdownPending &&
            now - pressedAt >= Shutdown) {
            shutdownPending = true;
            consumed = true;
        }
    }
};

static void pressRelease(Gesture& g, uint32_t start, uint32_t duration, bool calibrated = true, bool neutral = true) {
    g.sample(false, start);
    g.sample(true, start + 30);
    g.sample(true, start + 30 + duration);
    g.sample(false, start + 30 + duration + 30);
}

int main() {
    { Gesture g; g.sample(true, 0); pressRelease(g, 100, 800); assert(g.state == State::Locked); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 800); assert(g.state == State::Ready); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 200); assert(g.state == State::Locked); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 800); assert(g.state == State::Ready); pressRelease(g, 2000, 800); assert(g.state == State::Locked); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 800); assert(g.state == State::Ready); g.state = State::ControlledStop; pressRelease(g, 2000, 800); assert(g.state == State::ControlledStop); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 3000); assert(g.state == State::Locked); assert(g.shutdownPending == false); }
    { Gesture g; g.sample(false, 0); pressRelease(g, 100, 800, true, false); assert(g.state == State::Locked); pressRelease(g, 2000, 800); assert(g.state == State::Ready); }
    { Gesture g; g.sample(false, 0); g.state = State::Locked; pressRelease(g, 100, 800); assert(g.state == State::Ready); }
    return 0;
}
