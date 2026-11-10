#pragma once
#include <stdint.h>

// Small policy helper for the handset OTA maintenance interlock.
// The caller provides synchronization if the latch is shared across tasks.
class TorqueDollyMaintenanceLatch {
public:
    bool enter(bool handsetLocked) {
        if (!handsetLocked || active_ || transferActive_) return false;
        active_ = true;
        requireButtonRelease_ = true;
        return true;
    }

    bool exit() {
        if (transferActive_) return false;
        if (active_) {
            active_ = false;
            exitLockPending_ = true;
            requireButtonRelease_ = true;
        }
        return true;
    }

    bool beginTransfer() {
        if (!active_ || transferActive_ || exitLockPending_) return false;
        transferActive_ = true;
        return true;
    }

    bool finishTransfer() { transferActive_ = false; return true; }
    bool transferActive() const { return transferActive_; }

    bool active() const { return active_; }
    bool lockingAfterExit() const { return exitLockPending_; }

    void acknowledgeLocked() {
        exitLockPending_ = false;
    }

    bool blocksUnlock(bool buttonPressed) {
        if (!active_ && !exitLockPending_ && !buttonPressed) requireButtonRelease_ = false;
        return active_ || exitLockPending_ || requireButtonRelease_;
    }
    bool requiresNeutralOutput() const { return active_ || transferActive_ || exitLockPending_; }
    bool suspendsAutoPowerOff() const { return active_ || transferActive_ || exitLockPending_; }

    uint32_t channelOutput(uint32_t requested, uint32_t neutral) const {
        return requiresNeutralOutput() ? neutral : requested;
    }

private:
    bool active_ = false;
    bool transferActive_ = false;
    bool exitLockPending_ = false;
    bool requireButtonRelease_ = false;
};
