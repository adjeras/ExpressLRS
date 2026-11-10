#include <cassert>
#include <cstdint>
#include <cstdio>

#include "TorqueDollyButton.h"
#include "TorqueDollyMaintenance.h"

namespace {
constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t UNLOCK_MIN_MS = 700;
constexpr uint32_t SHUTDOWN_MS = 3000;

void update(TorqueDollyButton::Input &input, uint32_t now, bool raw)
{
    input.update(raw, now, DEBOUNCE_MS, SHUTDOWN_MS);
}

TorqueDollyButton::Action runGesture(
    TorqueDollyButton::Input &input,
    TorqueDollyButton::Gesture &buttonGesture,
    uint32_t rawPressAt,
    uint32_t rawHoldMs,
    bool canUnlock = true)
{
    update(input, 0, false);
    update(input, DEBOUNCE_MS, false);
    buttonGesture.update(input.snapshot, TorqueDollyButton::State::Locked,
                         canUnlock, UNLOCK_MIN_MS, SHUTDOWN_MS);

    update(input, rawPressAt, true);
    update(input, rawPressAt + DEBOUNCE_MS, true);
    buttonGesture.update(input.snapshot, TorqueDollyButton::State::Locked,
                         canUnlock, UNLOCK_MIN_MS, SHUTDOWN_MS);

    const uint32_t rawReleaseAt = rawPressAt + rawHoldMs;
    const uint32_t acceptedReleaseAt = rawReleaseAt + DEBOUNCE_MS;
    update(input, rawReleaseAt, false);
    update(input, acceptedReleaseAt, false);
    return buttonGesture.update(input.snapshot, TorqueDollyButton::State::Locked,
                               canUnlock, UNLOCK_MIN_MS, SHUTDOWN_MS);
}

void testUnlockBoundaries()
{
    for (const uint32_t rawHoldMs : {699U, 700U})
    {
        TorqueDollyButton::Input input;
        TorqueDollyButton::Gesture buttonGesture;
        const auto action = runGesture(input, buttonGesture, 100, rawHoldMs);

        assert(input.snapshot.duration == rawHoldMs);
        assert(action == (rawHoldMs == 700
            ? TorqueDollyButton::Action::Unlock
            : TorqueDollyButton::Action::None));
    }
}

void testShutdownBoundaries()
{
    for (const uint32_t rawHoldMs : {2999U, 3000U})
    {
        TorqueDollyButton::Input input;
        TorqueDollyButton::Gesture buttonGesture;
        const auto action = runGesture(input, buttonGesture, 100, rawHoldMs);

        assert(input.snapshot.duration == rawHoldMs);
        assert(action == TorqueDollyButton::Action::None);
        assert(input.snapshot.pending == (rawHoldMs == 3000));
    }
}

void testCanUnlockStillBlocks()
{
    TorqueDollyButton::Input input;
    TorqueDollyButton::Gesture buttonGesture;
    const auto action = runGesture(input, buttonGesture, 100, 700, false);

    assert(input.snapshot.duration == 700);
    assert(action == TorqueDollyButton::Action::None);
}

void testMaintenanceRequiresLockedAndAlwaysNeutral()
{
    TorqueDollyMaintenanceLatch maintenance;
    assert(!maintenance.enter(false));
    assert(!maintenance.active());
    assert(maintenance.enter(true));
    assert(maintenance.active());
    assert(maintenance.blocksUnlock(true));
    assert(maintenance.requiresNeutralOutput());
    assert(maintenance.suspendsAutoPowerOff());
    assert(maintenance.channelOutput(1000, 992) == 992);
    assert(maintenance.channelOutput(100, 992) == 992);
    assert(!maintenance.enter(true));
    maintenance.exit();
    assert(!maintenance.active());
    assert(maintenance.lockingAfterExit());
    assert(maintenance.requiresNeutralOutput());
    assert(maintenance.channelOutput(1000, 992) == 992);
    assert(maintenance.blocksUnlock(true)); // Require release after maintenance.
    maintenance.acknowledgeLocked(); // Handset loop emitted the neutral frame.
    assert(!maintenance.lockingAfterExit());
    assert(!maintenance.blocksUnlock(false));
    assert(maintenance.channelOutput(1000, 992) == 1000);
    // The latch has no Ready state to restore: its caller exits to Locked.
}
}

int main()
{
    testUnlockBoundaries();
    testShutdownBoundaries();
    testCanUnlockStillBlocks();
    testMaintenanceRequiresLockedAndAlwaysNeutral();
    std::puts("PASS: production button and handset maintenance latch safety tests");
    return 0;
}
