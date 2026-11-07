#include <cassert>
#include <cstdint>
#include <cstdio>

#include "TorqueDollyButton.h"

namespace {
constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t UNLOCK_MIN_MS = 700;
constexpr uint32_t SHUTDOWN_MS = 3000;

void update(TorqueDollyButton::Input &input, uint32_t now, bool raw)
{
    input.update(raw, now, DEBOUNCE_MS, SHUTDOWN_MS);
}

TorqueDollyButton::Action gesture(
    TorqueDollyButton::Input &input,
    TorqueDollyButton::Gesture &gesture,
    uint32_t rawPressAt,
    uint32_t rawHoldMs,
    bool canUnlock = true)
{
    update(input, 0, false);
    update(input, DEBOUNCE_MS, false);
    gesture.update(input.snapshot, TorqueDollyButton::State::Locked, canUnlock,
                   UNLOCK_MIN_MS, SHUTDOWN_MS);

    update(input, rawPressAt, true);
    update(input, rawPressAt + DEBOUNCE_MS, true);
    gesture.update(input.snapshot, TorqueDollyButton::State::Locked, canUnlock,
                   UNLOCK_MIN_MS, SHUTDOWN_MS);

    const uint32_t rawReleaseAt = rawPressAt + rawHoldMs;
    update(input, rawReleaseAt, false);
    update(input, rawReleaseAt + DEBOUNCE_MS, false);
    return gesture.update(input.snapshot, TorqueDollyButton::State::Locked,
                          canUnlock, UNLOCK_MIN_MS, SHUTDOWN_MS);
}

void testRawHoldAndDebouncedHold()
{
    TorqueDollyButton::Input input;
    TorqueDollyButton::Gesture gesture;

    // Raw hold is 670 ms, but debounced hold is 700 ms: unlock qualifies.
    const auto action = gesture(input, gesture, 100, 670);
    assert(action == TorqueDollyButton::Action::Unlock);
    assert(input.snapshot.duration == 700);
}

void testBelowDebouncedUnlockBoundary()
{
    TorqueDollyButton::Input input;
    TorqueDollyButton::Gesture gesture;

    // Raw hold is 669 ms, debounced hold is 699 ms: unlock is rejected.
    const auto action = gesture(input, gesture, 100, 669);
    assert(action == TorqueDollyButton::Action::None);
    assert(input.snapshot.duration == 699);
}

void testShutdownAndPendingPriority()
{
    TorqueDollyButton::Input input;
    TorqueDollyButton::Gesture gesture;

    const auto action = gesture(input, gesture, 100, 2970);
    assert(action == TorqueDollyButton::Action::None);
    assert(input.snapshot.duration == 3000);
    assert(input.snapshot.pending);
}

void testCanUnlockStillBlocks()
{
    TorqueDollyButton::Input input;
    TorqueDollyButton::Gesture gesture;

    const auto action = gesture(input, gesture, 100, 670, false);
    assert(action == TorqueDollyButton::Action::None);
    assert(input.snapshot.duration == 700);
}
}

int main()
{
    testRawHoldAndDebouncedHold();
    testBelowDebouncedUnlockBoundary();
    testShutdownAndPendingPriority();
    testCanUnlockStillBlocks();
    std::puts("PASS: production button debounce, unlock, shutdown, and eligibility tests");
    return 0;
}
