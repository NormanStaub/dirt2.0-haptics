"""
DualSense HD haptics synthesizer - layered feedback.

Over USB the DualSense is a 4-channel sound card: channels 1-2 feed the
speaker/headphone jack, channels 3-4 drive the left/right voice-coil
actuators directly.

Each piece of information gets its own layer with its own *feel* (frequency
band + rhythm), so they stay distinguishable when stacked. All layers are
textured noise (no beeps/sines) with slow random wander, so it reads as the
car rather than as effects.

  1 SPEED     faint, steady rolling texture. Pitch rises with speed; level
              barely changes, so speed is a background sense, not a buzz.
  2 SURFACE   the road's character. Tarmac: smooth and quiet. Gravel: coarse
              grain plus scattered stone ticks. Holds steady per surface.
  3 SLIDE     the car going sideways: a heavy, low, wallowing drag with
              irregular "grabs" (tyres catching and letting go). While sliding,
              the road layers DUCK (fade back) - losing grip feels like losing
              touch with the road.
  4 SPIN      wheelspin: a light, fast, fluttering buzz - high and thin, so it
              never gets confused with a slide.
  5 LOCK      locked wheels: road texture freezes out and a coarse skid with
              skips takes over.
  6 IMPACT    big hits / landings only: a dull thud on the side that took it.
  7 DRIVETRAIN  nothing at normal revs; swells near the redline, ragged on
              the limiter, short clunk on shifts.
"""

import threading

import numpy as np
import sounddevice as sd

SR = 48000
BLOCK = 256          # ~5 ms


def find_device():
    """WASAPI output of the DualSense with >= 4 channels, else None."""
    best = None
    for i, d in enumerate(sd.query_devices()):
        name = d["name"].lower()
        if d["max_output_channels"] < 4:
            continue
        if "dualsense" not in name and "wireless controller" not in name:
            continue
        api = sd.query_hostapis(d["hostapi"])["name"]
        if "WASAPI" in api:
            return i
        if best is None and "WDM" not in api:
            best = i
    return best


def _band_noise(lo, hi, seconds=4.0, seed=0):
    """Loopable band-limited noise table, normalised to +-1."""
    n = int(SR * seconds)
    rng = np.random.default_rng(seed)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / SR)
    spec[(f < lo) | (f > hi)] = 0
    x = np.fft.irfft(spec, n)
    return (x / (np.max(np.abs(x)) + 1e-9)).astype(np.float32)


class _Looper:
    """Plays a noise table at a variable rate with linear interpolation."""

    def __init__(self, table, seed=0):
        self.t = table
        self.n = len(table)
        self.pos = float(np.random.default_rng(seed).integers(0, self.n))

    def block(self, rate, frames):
        rate = max(0.02, rate)
        idx = self.pos + np.arange(frames) * rate
        self.pos = (self.pos + frames * rate) % self.n
        i0 = idx.astype(np.int64) % self.n
        frac = (idx - np.floor(idx)).astype(np.float32)
        return self.t[i0] * (1 - frac) + self.t[(i0 + 1) % self.n] * frac


class _Ticks:
    """Random short transients (stones / grit), Poisson-timed."""

    def __init__(self, seed, length=0.012, decay=0.0035, lo=120, hi=320):
        self.rng = np.random.default_rng(seed)
        n = int(SR * length)
        kt = np.arange(n, dtype=np.float32) / SR
        k = _band_noise(lo, hi, seconds=0.25, seed=seed + 100)[:n]
        self.kernel = (k * np.exp(-kt / decay)).astype(np.float32)
        self.kernel /= np.max(np.abs(self.kernel)) + 1e-9
        self.tail = np.zeros(n - 1, np.float32)

    def block(self, rate_hz, frames):
        imp = np.zeros(frames, np.float32)
        k = self.rng.poisson(max(0.0, rate_hz) * frames / SR)
        if k:
            pos = self.rng.integers(0, frames, k)
            np.add.at(imp, pos, self.rng.uniform(0.3, 1.0, k).astype(np.float32))
        y = np.convolve(imp, self.kernel)
        y[: len(self.tail)] += self.tail
        self.tail = y[frames:].copy()
        return y[:frames]


def _pair(lo, hi, seed, seconds=4.0):
    return [_Looper(_band_noise(lo, hi, seconds, seed), seed),
            _Looper(_band_noise(lo, hi, seconds, seed + 50), seed + 50)]


DEFAULTS = dict(
    speed_rate=1.0, speed_amp=0.0,          # 1 speed
    surface=0.0, surface_amp=0.0,           # 2 surface (0 tarmac .. 1 gravel)
    tick_rate=0.0, tick_amp=0.0,
    slide=0.0, slide_amp=0.0, slide_bias=0.0,   # 3 slide (bias -1 left .. +1 right)
    spin=0.0, spin_amp=0.0, spin_rate=1.0,  # 4 spin
    lock=0.0, lock_amp=0.0,                 # 5 lock
    engine_amp=0.0, engine_rate=1.0, limiter=False,  # 7 drivetrain
)


class Haptics:
    def __init__(self, gain=1.0, device=None):
        self.gain = gain
        self.device = find_device() if device is None else device
        if self.device is None:
            raise RuntimeError("DualSense audio device not found (haptics need a USB cable)")
        self.lock = threading.Lock()
        self.p = dict(DEFAULTS)
        self.prev = dict(DEFAULTS)
        self.impact = np.zeros(2, np.float32)
        self.clunk = 0.0

        self.speed_src = _pair(20, 60, 1)            # soft rolling
        self.tarmac = _pair(30, 90, 3)               # smooth
        self.gravel = _pair(45, 180, 5)              # coarse grain
        self.ticks = [_Ticks(11), _Ticks(12)]
        self.slide_src = _pair(14, 38, 7)            # heavy, low wallow
        self.grab = _pair(3, 11, 9, seconds=8)       # irregular stick-slip rhythm
        self.spin_src = _pair(150, 260, 13)          # thin, high buzz
        self.flutter = _pair(18, 35, 15)             # fast flutter on the buzz
        self.skid = _pair(40, 100, 17)               # coarse lock-up skid
        self.skip = _Looper(_band_noise(6, 16, 4, 19), 19)
        self.wander = _pair(0.3, 3, 21, seconds=8)   # organic drift on everything
        self.thud = _pair(25, 70, 23)
        self.engine = _Looper(_band_noise(90, 130, 4, 25), 25)
        self.fuelcut = _Looper(_band_noise(8, 18, 4, 27), 27)
        self.clunk_src = _Looper(_band_noise(30, 110, 4, 29), 29)

        self.stream = sd.OutputStream(device=self.device, samplerate=SR, channels=4,
                                      blocksize=BLOCK, dtype="float32",
                                      latency="low", callback=self._cb)

    # ---- called from telemetry thread ----
    def update(self, **kw):
        with self.lock:
            self.p.update(kw)

    def hit(self, left=0.0, right=0.0):
        with self.lock:
            self.impact[0] = max(self.impact[0], min(1.0, left))
            self.impact[1] = max(self.impact[1], min(1.0, right))

    def shift(self, strength=0.7):
        with self.lock:
            self.clunk = max(self.clunk, strength)

    def silence(self):
        self.update(speed_amp=0.0, surface_amp=0.0, tick_amp=0.0, slide_amp=0.0,
                    spin_amp=0.0, lock_amp=0.0, engine_amp=0.0, limiter=False)

    def start(self):
        self.stream.start()

    def close(self):
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass

    # ---- audio thread ----
    def _cb(self, out, frames, _time, _status):
        with self.lock:
            p = dict(self.p)
            imp = self.impact.copy()
            clunk = self.clunk
        q = self.prev
        ramp = np.linspace(0, 1, frames, endpoint=False, dtype=np.float32)

        def g(key):  # glide a parameter across the block (no zipper noise)
            return q[key] + (p[key] - q[key]) * ramp

        t = np.arange(frames, dtype=np.float32) / SR
        rate = max(0.2, p["speed_rate"])
        surf = float(min(1.0, max(0.0, p["surface"])))
        bias = float(max(-1.0, min(1.0, p["slide_bias"])))
        # road layers fade back while sliding / locked
        duck = np.clip(1.0 - 0.6 * g("slide") - 0.85 * g("lock"), 0.1, 1.0)
        skip = (self.skip.block(1.0, frames) > -0.2).astype(np.float32)

        hands = []
        for s in (0, 1):
            w = 1.0 + 0.2 * self.wander[s].block(1.0, frames)
            outside = 1.0 + (bias if s == 1 else -bias) * 0.35

            speed = self.speed_src[s].block(rate, frames) * g("speed_amp")
            road = (self.tarmac[s].block(rate, frames) * (1 - surf)
                    + self.gravel[s].block(rate, frames) * surf) * g("surface_amp")
            road = road + self.ticks[s].block(p["tick_rate"], frames) * g("tick_amp")

            grab = np.maximum(0.0, self.grab[s].block(1.0, frames)) * 1.6 + 0.35
            slide = self.slide_src[s].block(1.0, frames) * grab * g("slide_amp") * outside

            flut = 0.6 + 0.4 * self.flutter[s].block(1.0, frames)
            spin = self.spin_src[s].block(max(0.5, p["spin_rate"]), frames) * flut * g("spin_amp")

            lock = self.skid[s].block(1.0, frames) * skip * g("lock_amp")

            thud = self.thud[s].block(1.0, frames) * imp[s] * np.exp(-t / 0.07)

            hands.append((speed + road) * duck * w + slide * w + spin + lock + thud)

        eng = self.engine.block(max(0.2, p["engine_rate"]), frames) * g("engine_amp")
        if p["limiter"]:
            eng = eng * (0.25 + 1.5 * (self.fuelcut.block(1.0, frames) > 0.1))
        clk = self.clunk_src.block(1.0, frames) * clunk * np.exp(-t / 0.04)

        out[:, 0] = 0
        out[:, 1] = 0
        out[:, 2] = np.tanh((hands[0] + eng + clk) * self.gain)
        out[:, 3] = np.tanh((hands[1] + eng + clk) * self.gain)

        with self.lock:
            self.impact *= float(np.exp(-frames / SR / 0.07))
            self.clunk *= float(np.exp(-frames / SR / 0.04))
        self.prev = p
