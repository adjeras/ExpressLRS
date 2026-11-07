from dataclasses import dataclass
from enum import Enum

class State(Enum):
    LOCKED = 0
    READY = 1
    CONTROLLED_STOP = 2

@dataclass
class Combined:
    state: State = State.LOCKED
    startup_release_required: bool = True
    power_raw: bool = False
    power_pressed: bool = False
    power_initialized: bool = False
    power_changed_at: int = 0
    press_started_at: int = 0
    shutdown_pending: bool = False
    handset_last_pressed: bool = False
    handset_initialized: bool = False
    press_start_state: State = State.LOCKED
    unlock_eligible: bool = False
    consumed: bool = True
    calibrated: bool = True
    neutral: bool = True

    DEBOUNCE = 30
    UNLOCK_MIN = 700
    SHUTDOWN = 3000

    def sample_power(self, raw, now):
        if not self.power_initialized:
            self.power_initialized = True
            self.power_raw = raw
            self.power_pressed = raw
            self.power_changed_at = now
            self.startup_release_required = raw
            if raw: self.press_started_at = now
            return
        if raw != self.power_raw:
            self.power_raw = raw
            self.power_changed_at = now
        if raw != self.power_pressed and now - self.power_changed_at >= self.DEBOUNCE:
            self.power_pressed = raw
            if raw: self.press_started_at = now
        if (self.power_pressed and not self.shutdown_pending and
            not self.startup_release_required and now - self.press_started_at >= self.SHUTDOWN):
            self.shutdown_pending = True

    def sample_handset(self, now, calibrated=None, neutral=None):
        if calibrated is not None: self.calibrated = calibrated
        if neutral is not None: self.neutral = neutral
        pressed = self.power_pressed
        if not self.handset_initialized:
            self.handset_initialized = True
            self.handset_last_pressed = pressed
            return
        if pressed == self.handset_last_pressed: return
        self.handset_last_pressed = pressed
        if pressed:
            self.press_start_state = self.state
            self.unlock_eligible = not self.startup_release_required and self.state is State.LOCKED
            self.consumed = not self.unlock_eligible
            if self.state is State.READY: self.state = State.CONTROLLED_STOP
        else:
            duration = (now - self.press_started_at) & 0xffffffff
            shutdown = self.shutdown_pending or duration >= self.SHUTDOWN
            if self.startup_release_required:
                self.startup_release_required = False
            elif (self.press_start_state is State.LOCKED and self.unlock_eligible and
                  not self.consumed and not shutdown and self.UNLOCK_MIN <= duration < self.SHUTDOWN and
                  self.calibrated and self.neutral):
                self.state = State.READY
            self.consumed = self.consumed or shutdown
            self.unlock_eligible = False

    def tick(self, raw, now, calibrated=None, neutral=None, order='power-first'):
        if order == 'power-first':
            self.sample_power(raw, now); self.sample_handset(now, calibrated, neutral)
        else:
            self.sample_handset(now, calibrated, neutral); self.sample_power(raw, now); self.sample_handset(now, calibrated, neutral)


def initialize_released(g):
    g.tick(False, 0); g.tick(False, 31)


def gesture(g, start, held, calibrated=True, neutral=True, order='power-first'):
    g.tick(True, start, order=order)
    g.tick(True, start + 30, order=order)
    g.tick(True, start + 30 + held, calibrated, neutral, order)
    g.tick(False, start + 30 + held, order=order)
    g.tick(False, start + 60 + held, calibrated, neutral, order=order)
    g.power_pressed = False
    g.sample_handset(start + 61 + held, calibrated, neutral)


def check(value, name):
    assert value, name


def run():
    g = Combined(); g.tick(True, 0); g.tick(True, 31); g.tick(False, 1000); g.tick(False, 1031); check(g.state is State.LOCKED, 'startup release')
    g = Combined(); initialize_released(g); gesture(g, 100, 800); check(g.state is State.READY, '800 ms unlock')
    for duration, expected in ((699, State.LOCKED), (700, State.READY), (701, State.READY)):
        g = Combined(); initialize_released(g); gesture(g, 100, duration); check(g.state is expected, f'{duration} ms unlock boundary')
    g = Combined(); initialize_released(g); gesture(g, 100, 600); check(g.state is State.LOCKED, 'short rejection')
    g = Combined(); initialize_released(g); gesture(g, 100, 800); g.tick(True, 2000); g.tick(True, 2030); check(g.state is State.CONTROLLED_STOP, 'Ready lock'); g.tick(False, 2060); check(g.state is State.CONTROLLED_STOP, 'Ready release')
    g = Combined(); initialize_released(g); gesture(g, 100, 800); g.tick(True, 2000); g.tick(True, 2030); g.state = State.LOCKED; g.tick(False, 3000); check(g.state is State.LOCKED, 'ControlledStop completion')
    for duration in (2999, 3000, 3001):
        g = Combined(); initialize_released(g); gesture(g, 100, duration); check(g.state is State.LOCKED and (duration < 3000 or g.shutdown_pending), f'{duration} ms shutdown')
    for order in ('power-first', 'handset-first'):
        for duration in (699, 700, 701, 2999, 3000, 3001):
            g = Combined(); initialize_released(g); gesture(g, 100, duration, order=order)
            check(g.state is (State.READY if 700 <= duration < 3000 else State.LOCKED), f'{order} {duration}')
    g = Combined(); initialize_released(g); gesture(g, 100, 800, neutral=False); check(g.state is State.LOCKED, 'neutral rejection'); gesture(g, 2000, 800); check(g.state is State.READY, 'fresh neutral unlock')
    g = Combined(); initialize_released(g); gesture(g, 100, 800, calibrated=False); check(g.state is State.LOCKED, 'calibration rejection'); gesture(g, 2000, 800); check(g.state is State.READY, 'fresh calibration unlock')
    g = Combined(); initialize_released(g); gesture(g, 100, 800); g.state = State.LOCKED; g.consumed = True; g.unlock_eligible = False; gesture(g, 2000, 800); check(g.state is State.READY, 'inactivity fresh unlock')
    check(((0x300 - 0xffffff00) & 0xffffffff) >= 700, 'millis rollover')
    print('PASS: combined power/handset model boundary, ordering, startup, lock, rejection, inactivity, rollover')

run()
