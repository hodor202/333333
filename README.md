import time
import pigpio

DIR  = 20   # Direction pin
STEP = 21   # Step pin (any GPIO)
CW   = 1

# ---------- Motion profile (SAFE/slow) ----------
RUN_TIME_S   = 5.0      # total run time in one direction
RAMP_TIME_S  = 2.0      # longer ramp (gentler)
START_HZ     = 100      # very slow start to avoid stalls
TARGET_HZ    = 1200     # moderate cruise; raise later if stable (e.g., 2000)
RAMP_CHUNK_S   = 0.05   # 50 ms chunks during ramp (smoother)
CRUISE_CHUNK_S = 0.10   # 100 ms chunks during cruise

# If your driver has ENABLE (active low), set and wire it, then uncomment:
# ENABLE = 16
# USE_ENABLE = True
USE_ENABLE = False

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Start pigpio: sudo systemctl enable --now pigpiod")

# Pins
pi.set_mode(DIR, pigpio.OUTPUT)
pi.write(DIR, CW)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.write(STEP, 0)
if USE_ENABLE:
    pi.set_mode(ENABLE, pigpio.OUTPUT)
    pi.write(ENABLE, 0)  # enable driver (active low on DRV8825/A4988)

def make_wave_for_freq(hz: float, duration_s: float):
    """
    Create a wave for a 50% duty square on STEP at `hz` for about `duration_s`.
    Returns wave id; caller must delete it after sending.
    """
    hz = max(1.0, float(hz))
    half_us = int(0.5 / hz * 1_000_000)
    # Keep pulses >= 5 us to be well above the driver min (1.9–2.0 us)
    if half_us < 5:
        half_us = 5

    # number of full cycles ~ hz * duration
    cycles = max(1, int(hz * duration_s))
    pulses = []
    # Build: high half-period, low half-period, repeated
    set_mask = 1 << STEP
    for _ in range(cycles):
        pulses.append(pigpio.pulse(set_mask, 0, half_us))
        pulses.append(pigpio.pulse(0, set_mask, half_us))

    pi.wave_clear()
    pi.wave_add_generic(pulses)
    wid = pi.wave_create()
    return wid

def play_wave_blocking(wid):
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.001)
    pi.wave_delete(wid)

def self_test_single_freq(freq=400, steps=200):
    """Send a fixed number of steps at a safe frequency to prove motion."""
    # duration needed to emit 'steps' at 'freq'
    duration = steps / float(freq)  # seconds
    wid = make_wave_for_freq(freq, duration)
    play_wave_blocking(wid)

try:
    # ---------- 0) SELF-TEST ----------
    # Does the motor move ~half a rev at 400 Hz? (with 200 full steps/rev, no microstep)
    self_test_single_freq(freq=400, steps=200)

    # Small pause so you can see it stopped
    time.sleep(0.3)

    # ---------- 1) RAMP ----------
    t0 = time.perf_counter()
    if RAMP_TIME_S > 0:
        steps = max(1, int(RAMP_TIME_S / RAMP_CHUNK_S))
        for i in range(steps):
            a = (i + 1) / steps
            hz = START_HZ + a * (TARGET_HZ - START_HZ)
            wid = make_wave_for_freq(hz, RAMP_CHUNK_S)
            play_wave_blocking(wid)

    # ---------- 2) CRUISE until total RUN_TIME_S ----------
    elapsed = time.perf_counter() - t0
    remaining = max(0.0, RUN_TIME_S - elapsed)
    while remaining > 0:
        dur = CRUISE_CHUNK_S if remaining >= CRUISE_CHUNK_S else remaining
        wid = make_wave_for_freq(TARGET_HZ, dur)
        play_wave_blocking(wid)
        remaining -= dur

finally:
    pi.wave_tx_stop()
    pi.write(STEP, 0)
    if USE_ENABLE:
        pi.write(ENABLE, 1)  # disable if wired active low
    pi.stop()
