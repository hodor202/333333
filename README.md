import time
import pigpio

DIR  = 20
STEP = 21
CW   = 1

RUN_TIME_S     = 5.0
RAMP_TIME_S    = 2.0
START_HZ       = 100
TARGET_HZ      = 1200
RAMP_CHUNK_S   = 0.05
CRUISE_CHUNK_S = 0.10

USE_ENABLE = False
# ENABLE = 16  # define if you set USE_ENABLE = True

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Start pigpio: sudo systemctl enable --now pigpiod")

pi.set_mode(DIR, pigpio.OUTPUT)
pi.write(DIR, CW)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.write(STEP, 0)
if USE_ENABLE:
    pi.set_mode(ENABLE, pigpio.OUTPUT)
    pi.write(ENABLE, 0)

def make_wave_for_freq(hz: float, duration_s: float) -> int:
    hz = max(1.0, float(hz))
    half_us = int(0.5 / hz * 1_000_000)
    if half_us < 5:
        half_us = 5

    cycles = max(1, int(hz * duration_s))

    set_mask = 1 << STEP
    clr_mask = set_mask

    pulses = []
    for _ in range(cycles):
        pulses.append(pigpio.pulse(set_mask, 0,      half_us))
        pulses.append(pigpio.pulse(0,        clr_mask, half_us))

    if not pulses:
        raise RuntimeError("No pulses generated for wave")

    pi.wave_clear()
    pi.wave_add_generic(pulses)
    wid = pi.wave_create()

    if wid is None or not isinstance(wid, int) or wid < 0:
        raise RuntimeError(f"wave_create() failed, wid={wid}")

    return int(wid)

def play_wave_blocking(wid: int):
    wid = int(wid)
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.001)
    pi.wave_delete(wid)

def self_test_single_freq(freq=400, steps=200):
    duration = steps / float(freq)
    wid = make_wave_for_freq(freq, duration)
    play_wave_blocking(wid)

try:
    self_test_single_freq(freq=400, steps=200)
    time.sleep(0.3)

    t0 = time.perf_counter()
    if RAMP_TIME_S > 0:
        steps = max(1, int(RAMP_TIME_S / RAMP_CHUNK_S))
        for i in range(steps):
            a = (i + 1) / steps
            hz = START_HZ + a * (TARGET_HZ - START_HZ)
            wid = make_wave_for_freq(hz, RAMP_CHUNK_S)
            play_wave_blocking(wid)

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
        pi.write(ENABLE, 1)
    pi.stop()
