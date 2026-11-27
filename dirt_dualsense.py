"""
DiRT Rally 2.0 -> DualSense bridge.

Reads DiRT Rally 2.0's UDP telemetry (extradata="3") and drives the PS5
controller's adaptive triggers, rumble, lightbar and player LEDs.

  R2 (throttle): free trigger, small brief push on each gear change.
  L2 (brake):    free, then lightest resistance (1/8) in the bottom quarter;
                 faint judder when wheels lock.
  HD haptics:    (USB) layered: faint speed, surface (tarmac vs gravel),
                 a distinct heavy drag when sliding (road fades back),
                 thin buzz on wheelspin, skid on lock-up, thuds on big
                 hits, drivetrain near the redline, shift clunks.
                 See haptics.py. Falls back to rumble motors over Bluetooth.
  Lightbar:      green -> yellow -> red shift light, flashes at the limiter.
  Player LEDs:   5-segment rev bar.

Usage:
  python dirt_dualsense.py            run the bridge (Ctrl+C to stop)
  python dirt_dualsense.py --test     cycle effects on the controller, no game needed
  python dirt_dualsense.py --help     tuning options

Needs:  pip install hidapi sounddevice numpy
Game:   Documents\\My Games\\DiRT Rally 2.0\\hardwaresettings\\hardware_settings_config.xml
        <udp enabled="true" extradata="3" ip="127.0.0.1" port="20777" delay="1" />
"""

import argparse
import math
import socket
import struct
import threading
import sys
import time
import zlib

import hid

SONY_VID = 0x054C
DUALSENSE_PIDS = {0x0CE6, 0x0DF2}  # DualSense, DualSense Edge


# --------------------------------------------------------------------------
# Adaptive trigger effects (11-byte blocks).
# Zones: the trigger travel is split into 10 zones (0 = top, 9 = fully pressed).
# Strength/amplitude per zone is 0 (inactive) .. 8.
# --------------------------------------------------------------------------

def trig_off():
    return bytes([0x05] + [0] * 10)


def _zones(strengths):
    active = 0
    packed = 0
    for i, s in enumerate(strengths[:10]):
        s = int(max(0, min(8, round(s))))
        if s > 0:
            active |= 1 << i
            packed |= ((s - 1) & 0x7) << (3 * i)
    return active, packed


def trig_feedback(strengths):
    """Resistance; strengths = 10 values 0..8, one per zone of travel."""
    active, packed = _zones(strengths)
    if not active:
        return trig_off()
    return bytes([0x21, active & 0xFF, (active >> 8) & 0xFF,
                  packed & 0xFF, (packed >> 8) & 0xFF,
                  (packed >> 16) & 0xFF, (packed >> 24) & 0xFF,
                  0, 0, 0, 0])


def trig_vibration(amplitudes, freq):
    """Vibrating trigger; amplitudes = 10 values 0..8, freq in Hz (1..255)."""
    active, packed = _zones(amplitudes)
    freq = int(max(1, min(255, freq)))
    if not active:
        return trig_off()
    return bytes([0x26, active & 0xFF, (active >> 8) & 0xFF,
                  packed & 0xFF, (packed >> 8) & 0xFF,
                  (packed >> 16) & 0xFF, (packed >> 24) & 0xFF,
                  0, 0, freq, 0])


def ramp(start_zone, s_from, s_to):
    """Zones before start_zone free, then strength rising s_from -> s_to."""
    out = [0] * 10
    n = 10 - start_zone
    for k in range(n):
        t = k / max(1, n - 1)
        out[start_zone + k] = max(1, s_from + (s_to - s_from) * t)
    return out


def flat(start_zone, s):
    return [0] * start_zone + [s] * (10 - start_zone)


# --------------------------------------------------------------------------
# Controller output
# --------------------------------------------------------------------------

class DualSense:
    """Controller output. Reports are written from a background thread (a USB
    write blocks ~5 ms), only when something changed, and always the newest
    state - so telemetry handling never waits on the controller."""

    def __init__(self):
        self.dev = None
        self.bt = False
        self.seq = 0
        self.last_try = 0.0
        self.last_common = None
        self.last_queued = 0.0
        self._slot = None
        self._cv = threading.Condition()
        self._running = True
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

    def connect(self):
        now = time.time()
        if self.dev or now - self.last_try < 2.0:
            return self.dev is not None
        self.last_try = now
        for info in hid.enumerate(SONY_VID):
            if info["product_id"] not in DUALSENSE_PIDS:
                continue
            try:
                d = hid.device()
                d.open_path(info["path"])
            except OSError:
                continue
            bus = info.get("bus_type")
            self.bt = (bus == 2) if bus is not None else info["interface_number"] == -1
            self.dev = d
            self.last_common = None
            print(f"[controller] connected over {'Bluetooth' if self.bt else 'USB'}")
            return True
        return False

    def _report(self, c):
        if self.bt:
            r = bytearray(78)
            r[0] = 0x31
            r[1] = (self.seq & 0x0F) << 4
            self.seq = (self.seq + 1) & 0x0F
            r[2] = 0x10
            r[3:3 + 47] = c
            crc = zlib.crc32(b"\xA2" + bytes(r[:74])) & 0xFFFFFFFF
            r[74:78] = struct.pack("<I", crc)
            return bytes(r)
        return b"\x02" + bytes(c)

    def _write(self, c):
        dev = self.dev
        if dev is None:
            return
        try:
            dev.write(self._report(c))
        except (OSError, ValueError):
            print("[controller] lost connection, retrying...")
            try:
                dev.close()
            except Exception:
                pass
            self.dev = None

    def _write_loop(self):
        while True:
            with self._cv:
                while self._slot is None and self._running:
                    self._cv.wait()
                if self._slot is None and not self._running:
                    return
                c, self._slot = self._slot, None
            self._write(c)

    def send(self, rumble_low=0, rumble_high=0, r2=None, l2=None,
             rgb=(0, 0, 64), player_leds=0, haptics=False, force=False):
        if not self.connect():
            return
        c = bytearray(47)
        c[0] = 0x04 | 0x08                  # R2, L2 trigger effects
        if not haptics:
            c[0] |= 0x01 | 0x02             # compat (rumble-motor) vibration
        c[1] = 0x04 | 0x10                  # lightbar, player LEDs
        c[2] = int(max(0, min(255, rumble_high)))  # right motor (small / buzzy)
        c[3] = int(max(0, min(255, rumble_low)))   # left motor (big / thumpy)
        c[10:21] = r2 or trig_off()
        c[21:32] = l2 or trig_off()
        if not haptics:
            c[38] = 0x04                    # compat vibration (newer firmware)
        c[43] = player_leds & 0x1F
        c[44], c[45], c[46] = (int(max(0, min(255, v))) for v in rgb)

        now = time.time()
        # unchanged state: skip, but refresh twice a second in case something
        # else (e.g. Steam) touched the controller
        if not force and c == self.last_common and now - self.last_queued < 0.5:
            return
        self.last_common = bytes(c)
        self.last_queued = now
        with self._cv:
            self._slot = bytes(c)           # newest wins
            self._cv.notify()

    def reset(self):
        self.send(force=True)

    def close(self):
        if self.dev:
            self.reset()
        with self._cv:
            self._running = False
            self._cv.notify()
        self._writer.join(timeout=1.0)
        if self.dev:
            self.dev.close()
            self.dev = None


# --------------------------------------------------------------------------
# Telemetry (Codemasters "extradata=3" packet: 66 little-endian floats)
# --------------------------------------------------------------------------

class Telemetry:
    __slots__ = ("speed", "velocity", "axis_a", "axis_b", "susp_pos", "susp_vel",
                 "wheel_speed", "throttle", "steer", "brake",
                 "clutch", "gear", "g_lat", "g_lon", "rpm", "max_rpm", "idle_rpm")

    @classmethod
    def parse(cls, data):
        if len(data) < 64 * 4:
            return None
        n = min(66, len(data) // 4)
        f = struct.unpack_from(f"<{n}f", data)
        t = cls()
        t.speed = f[7]                       # m/s
        t.velocity = f[8:11]                 # world-space velocity
        t.axis_a = f[11:14]                  # car "roll" vector
        t.axis_b = f[14:17]                  # car "pitch" vector
        t.susp_pos = f[17:21]                # RL RR FL FR
        t.susp_vel = f[21:25]                # RL RR FL FR
        t.wheel_speed = f[25:29]             # RL RR FL FR, m/s
        t.throttle, t.steer, t.brake, t.clutch = f[29], f[30], f[31], f[32]
        t.gear = int(round(f[33]))           # -1/10 = reverse, 0 = neutral
        t.g_lat, t.g_lon = f[34], f[35]
        t.rpm = f[37]
        t.max_rpm = f[63] if n > 63 else 0.0
        t.idle_rpm = f[64] if n > 64 else 0.0
        return t


# --------------------------------------------------------------------------
# Effect logic
# --------------------------------------------------------------------------

# Brake resistance per zone of travel (0 = free, 8 = max). The hardware has
# whole steps only: free for most of the travel, then 2/8 in the bottom
# quarter (last ~25-30%).
BRAKE_CURVE = [0, 0, 0, 0, 0, 0, 0, 2, 2, 2]
BRAKE_JUDDER = 2                    # steps added on alternate beats when wheels lock
BRAKE_START = 7


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


class Effects:
    def __init__(self, args, haptics=None):
        self.a = args
        self.hx = haptics
        self.last_gear = None
        self.shift_until = 0.0
        self.shift_blip_until = 0.0
        self.prev_g_lon = 0.0
        self.hit = 0.0
        self.peak_rpm = 1.0
        self.prev_susp_vel = None
        self.prev_time = None
        self.susp_unit = None          # 1.0 if metres, 0.001 if millimetres
        self.hit_cooldown = [0.0, 0.0]
        self.prev_fwd = None
        self.hs = dict(tex=0.0, angle=0.0, mom=0.0, vy=0.0, air_until=0.0, air_start=0.0,
                       was_air=0.0, decel=0.0, prev_speed=0.0, big_cooldown=0.0, slide=0.0, bias=0.0, spin=0.0, lock=0.0,
                       svlp0=0.0, svlp1=0.0, svlp2=0.0, svlp3=0.0)

    def compute(self, t, now):
        a = self.a
        dt = min(0.1, max(1 / 240, now - self.prev_time)) if self.prev_time else 1 / 60
        self.prev_time = now

        # RPM fraction (some builds report max_rpm 0 -> learn it)
        self.peak_rpm = max(self.peak_rpm, t.rpm)
        max_rpm = t.max_rpm if t.max_rpm > 1 else self.peak_rpm
        idle = t.idle_rpm if 0 < t.idle_rpm < max_rpm else 0.0
        rpm_frac = max(0.0, min(1.0, (t.rpm - idle) / max(1.0, max_rpm - idle)))
        real_rpm = t.rpm * (10 if max_rpm < 2000 else 1)   # DR2 reports rpm / 10

        # Suspension units: DR2 sends mm on some builds, m on others
        if self.susp_unit is None or max(abs(p) for p in t.susp_pos) > 2.0:
            self.susp_unit = 0.001 if max(abs(p) for p in t.susp_pos) > 2.0 else 1.0
        sv = [v * self.susp_unit for v in t.susp_vel]   # m/s, RL RR FL FR

        # Slip detection from wheel speeds vs car speed (overall + per side)
        speed = abs(t.speed)
        ws = [abs(w) for w in t.wheel_speed]
        ref = max(speed, 4.0)
        spin = max(0.0, (max(ws) - speed) / ref)
        lock = max(0.0, (speed - min(ws)) / ref) if speed > 3.0 else 0.0
        side_slip = []
        for pair in ((ws[0], ws[2]), (ws[1], ws[3])):          # left, right wheels
            s = max(abs(w - speed) for w in pair) / ref if speed > 1.0 else 0.0
            side_slip.append(s)

        # Sideways sliding: the smaller of the two body-axis components of velocity
        da = abs(_dot(t.velocity, t.axis_a))
        db = abs(_dot(t.velocity, t.axis_b))
        lateral = min(da, db) / ref if speed > 3.0 else 0.0

        # ---- R2: throttle - slight resistance, firm blip on gear changes ----
        if now < self.shift_blip_until:
            r2 = trig_feedback(flat(0, a.gas_blip))               # small push on shift
        else:
            r2 = trig_off()                                       # free trigger

        # ---- L2: brake - light ramp, gentle judder on lock ----
        if t.brake > 0.05 and lock > a.lock_threshold:
            # Faint judder: same resistance ramp, nudged +1 on alternate
            # ~40 ms steps (a vibration effect would drop the resistance).
            bump = BRAKE_JUDDER if int(now * 25) % 2 else 0
            l2 = trig_feedback(BRAKE_CURVE[:BRAKE_START] + [BRAKE_CURVE[-1] + bump] * (10 - BRAKE_START))
        else:
            l2 = trig_feedback(BRAKE_CURVE)

        # ---- Rumble ----
        shifted = self.last_gear is not None and t.gear != self.last_gear
        if shifted:
            self.shift_until = now + 0.08
            self.shift_blip_until = now + 0.15
        self.last_gear = t.gear

        susp = sum(abs(v) for v in sv) / 4.0                  # m/s
        g_jolt = abs(t.g_lon - self.prev_g_lon)
        self.prev_g_lon = t.g_lon
        self.hit = max(self.hit * 0.80, min(1.0, susp * 0.6 + g_jolt * 0.5))

        if self.hx:
            if shifted:
                self.hx.shift(0.8)
            self._haptics(t, now, dt, sv, speed, rpm_frac, real_rpm,
                          spin, lock, g_jolt)
            low = high = 0
        else:
            low = 255 * self.hit
            high = 255 * (0.10 * rpm_frac * t.throttle + min(0.6, (spin + lock) * 1.5))
            if now < self.shift_until:
                low = max(low, 200)
                high = max(high, 120)
            if speed < 0.5 and t.rpm < 1:
                low = high = 0
            low *= a.rumble
            high *= a.rumble

        # ---- Lightbar / player LEDs ----
        if a.no_lights:
            rgb, leds = (0, 0, 64), 0
        else:
            if rpm_frac > 0.97:
                rgb = (255, 0, 0) if int(now * 12) % 2 else (40, 0, 0)
            elif rpm_frac > 0.85:
                rgb = (255, 30, 0)
            elif rpm_frac > 0.70:
                rgb = (255, 160, 0)
            else:
                rgb = (0, int(60 + 160 * rpm_frac), 0)
            segs = int(rpm_frac * 5.999)
            leds = (0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F)[min(5, segs)]

        return dict(rumble_low=low, rumble_high=high, r2=r2, l2=l2,
                    rgb=rgb, player_leds=leds, haptics=self.hx is not None)

    # Calibrated from a recorded DiRT Rally 2.0 stage (gravel + a tarmac section):
    #   surface texture  ~0.05 tarmac, 0.15-0.24 gravel
    #   slip angle       p50 7 deg, p75 13 deg, p95 32 deg (rally = always a bit sideways)
    #   wheel/car speed  p50 +9 % (normal), real wheelspin 20-50 %
    TEX_TARMAC, TEX_GRAVEL = 0.08, 0.18
    SLIDE_START, SLIDE_FULL = 3.0, 40.0        # degrees: angle is felt across this whole range
    MOM_START, MOM_FULL = 0.5, 10.0            # m/s of sideways velocity
    AIR_DROOP = -140.0                         # mm: suspension hanging near full droop (-195)
    AIR_DROOP_SOFT = -110.0                    # mm: ...or extended while in free-fall
    SPIN_START, SPIN_FULL = 0.20, 0.50         # slip ratio
    LOCK_START, LOCK_FULL = 0.25, 0.60
    IMPACT_HP = 1.5                            # m/s of sudden suspension movement
    BIG_BUMP = 2.2                             # m/s suspension speed: heavy bump (p99.9 of a stage)
    BIG_BUMP_FULL = 5.0
    LANDING_AIRTIME = 0.10                     # s in the air before a landing gets a big hit
    CRASH_DECEL = 30.0                         # m/s2 (~3 g, beyond any braking)

    def _haptics(self, t, now, dt, sv, speed, rpm_frac, real_rpm,
                 spin, lock, g_jolt):
        """Telemetry -> layered haptics (see haptics.py)."""
        hx = self.hx
        st = self.hs

        def ema(key, value, tau):
            k = 1.0 - math.exp(-dt / tau)
            st[key] += (value - st[key]) * k
            return st[key]

        def ramp01(x, a, b):
            return min(1.0, max(0.0, (x - a) / (b - a)))

        # --- 2 SURFACE: fine (high-passed) suspension movement, averaged 0.5 s
        hp = []
        for i in range(4):
            low = ema(f"svlp{i}", sv[i], 0.05)
            hp.append(sv[i] - low)
        tex = ema("tex", sum(abs(h) for h in hp) / 4, 0.5)
        surface = ramp01(tex, self.TEX_TARMAC, self.TEX_GRAVEL)

        # --- body frame: axis_b = forward, axis_a = sideways (verified on recording)
        v_fwd = abs(_dot(t.velocity, t.axis_b))
        v_side = _dot(t.velocity, t.axis_a)
        slip_deg = math.degrees(math.atan2(abs(v_side), max(1.0, v_fwd))) if speed > 6 else 0.0

        # --- 3 SLIDE
        angle = ema("angle", ramp01(slip_deg, self.SLIDE_START, self.SLIDE_FULL) ** 0.7, 0.03)
        mom = ema("mom", ramp01(abs(v_side), self.MOM_START, self.MOM_FULL), 0.03)
        slide = angle * (0.35 + 0.65 * mom)        # how much "sideways" overall
        bias = ema("bias", (1.0 if v_side > 0 else -1.0) * min(1.0, slide * 3), 0.25)

        # --- AIRBORNE: all wheels hanging at droop, or extended + free-falling
        vy = t.velocity[1]
        ay = (vy - st["vy"]) / dt
        st["vy"] = vy
        sp = t.susp_pos
        droop = all(p < self.AIR_DROOP for p in sp)
        falling = all(p < self.AIR_DROOP_SOFT for p in sp) and -14.0 < ay < -6.5
        airborne = (droop or falling) and speed > 3
        touched = any(p > -100.0 for p in sp)          # a wheel is back on the ground
        if airborne:
            st["air_until"] = now + 0.06               # hold through flicker mid-jump
        if touched:
            st["air_until"] = 0.0                      # ...but release on touchdown
        air = 1.0 if now < st["air_until"] else 0.0

        # --- BIG HIT: landing after real airtime
        if air and not st["was_air"]:
            st["air_start"] = now
        if st["was_air"] and not air:
            airtime = now - st["air_start"]
            if airtime >= self.LANDING_AIRTIME:
                comp = max(sv)
                strength = 0.55 + 0.25 * ramp01(airtime, 0.1, 0.6) + 0.2 * ramp01(comp, 1.0, 3.0)
                hx.pulse(strength)
                st["big_cooldown"] = now + 0.3
        st["was_air"] = air

        # --- 4 SPIN / 5 LOCK from wheel speed vs forward speed
        ws = sum(abs(w) for w in t.wheel_speed) / 4
        ratio = (ws - v_fwd) / max(5.0, v_fwd)
        spin_amt = ramp01(ratio, self.SPIN_START, self.SPIN_FULL) if t.throttle > 0.3 else 0.0
        spin_amt = ema("spin", spin_amt, 0.03) * (1 - air)
        lock_amt = ramp01(-ratio, self.LOCK_START, self.LOCK_FULL) if t.brake > 0.2 else 0.0
        lock_amt = ema("lock", lock_amt, 0.025) * (1 - air)

        # --- 6 IMPACT: big sudden suspension movement, per side
        for s, (i, j) in enumerate(((0, 2), (1, 3))):
            h = max(abs(hp[i]), abs(hp[j]))
            if h > self.IMPACT_HP and now > self.hit_cooldown[s]:
                strength = min(1.0, 0.35 + (h - self.IMPACT_HP) * 0.8)
                hx.hit(**({"left": strength} if s == 0 else {"right": strength}))
                self.hit_cooldown[s] = now + 0.25
        if g_jolt > 2.5:
            hx.hit(left=min(1.0, g_jolt / 5), right=min(1.0, g_jolt / 5))

        # --- BIG HIT: heavy bumps (per side) and crashes
        decel = ema("decel", (st["prev_speed"] - speed) / dt, 0.03)
        if now > st["big_cooldown"] and not air:
            left = max(abs(sv[0]), abs(sv[2]))
            right = max(abs(sv[1]), abs(sv[3]))
            peak = max(left, right)
            if peak > self.BIG_BUMP:
                strength = 0.45 + 0.55 * ramp01(peak, self.BIG_BUMP, self.BIG_BUMP_FULL)
                hx.pulse(strength, left=0.6 + 0.4 * (left >= right), right=0.6 + 0.4 * (right > left))
                st["big_cooldown"] = now + 0.3
            if decel > self.CRASH_DECEL and st["prev_speed"] > 3:
                hx.pulse(0.6 + 0.4 * ramp01(decel, self.CRASH_DECEL, 80.0))
                st["big_cooldown"] = now + 0.4
        st["prev_speed"] = speed

        # --- 7 DRIVETRAIN: silent until near the redline
        firing = real_rpm / 60.0 * 2.0
        while firing > 220:
            firing /= 2
        eng_amp = 0.0                                  # RPM / redline not felt in the body

        # --- 1 SPEED
        moving = min(1.0, speed / 8.0)
        speed_rate = 0.4 + speed / 25.0

        g = self.a.rumble
        hx.update(speed_rate=speed_rate, speed_amp=0.025 * moving * g,
                  surface=surface,
                  surface_amp=(0.03 + 0.05 * surface) * moving * g,
                  tick_rate=surface * (10 + speed * 2.0),
                  tick_amp=0.16 * surface * moving * g,
                  slide=slide, slide_amp=slide * 0.45 * g, slide_bias=bias,
                  slide_angle=angle, slide_mom=mom, slide_grit=(0.15 + 0.35 * mom) * angle * g,
                  air=air,
                  spin=spin_amt, spin_amp=spin_amt * 0.30 * g,
                  spin_rate=0.8 + max(0.0, ratio) * 3.0,
                  lock=lock_amt, lock_amp=lock_amt * 0.35 * g,
                  engine_amp=eng_amp * g, engine_rate=firing / 110.0,
                  limiter=False)

# --------------------------------------------------------------------------

def run_bridge(args):
    pad = DualSense()
    if not pad.connect():
        print("[controller] no DualSense found yet, will keep looking...")
    hx = make_haptics(args, pad)
    fx = Effects(args, hx)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.05)
    print(f"[telemetry] listening on {args.ip}:{args.port}  (Ctrl+C to stop)")

    rec = None
    if args.record:
        import os
        rec_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
        os.makedirs(rec_dir, exist_ok=True)
        rec_path = os.path.join(rec_dir, time.strftime("stage_%Y%m%d_%H%M%S.bin"))
        rec = open(rec_path, "wb")
        print(f"[record] saving telemetry to {rec_path}")

    last_packet = 0.0
    idle_sent = False
    packets = 0
    try:
        while True:
            try:
                batch = [sock.recvfrom(2048)[0]]
            except socket.timeout:
                batch = []
            if batch:
                # drain anything else already queued so we never act on stale data
                sock.setblocking(False)
                try:
                    while len(batch) < 64:
                        batch.append(sock.recvfrom(2048)[0])
                except (BlockingIOError, OSError):
                    pass
                sock.settimeout(0.05)
            now = time.time()
            data = None
            out = None
            for data in batch:
                if rec:
                    rec.write(struct.pack("<dI", now, len(data)) + data)
                    if packets % 120 == 0:
                        rec.flush()
                t = Telemetry.parse(data)
                if t:
                    if not last_packet or now - last_packet > args.timeout:
                        print("[telemetry] receiving data from the game")
                    last_packet = now
                    idle_sent = False
                    packets += 1
                    out = fx.compute(t, now)        # every packet feeds the logic...
                    if args.verbose and packets % 30 == 0:
                        print(f"  speed {t.speed*3.6:5.0f} km/h  rpm {t.rpm:6.0f}  gear {t.gear:2d}  "
                              f"thr {t.throttle:.2f} brk {t.brake:.2f}")
            if out:
                pad.send(**out)                     # ...only the newest goes to the pad
            if not batch and last_packet and now - last_packet > args.timeout and not idle_sent:
                print("[telemetry] no data (paused / menu) - triggers released")
                pad.reset()
                if hx:
                    hx.silence()
                idle_sent = True
    except KeyboardInterrupt:
        pass
    finally:
        if rec:
            rec.close()
        if hx:
            hx.close()
        pad.close()
        sock.close()
        print("stopped, controller reset")


def make_haptics(args, pad):
    if args.no_haptics:
        print("[haptics] off (using rumble-motor mode)")
        return None
    if pad.bt:
        print("[haptics] Bluetooth connection - HD haptics need USB, using rumble-motor mode")
        return None
    try:
        from haptics import Haptics
        hx = Haptics(gain=args.haptics_gain)
        hx.start()
        print(f"[haptics] HD haptics on (audio device #{hx.device}, {hx.mode} mode, "
              f"{hx.stream.latency * 1000:.1f} ms output latency)")
        return hx
    except Exception as e:  # missing libs, device busy, no USB audio
        print(f"[haptics] unavailable ({e}) - using rumble-motor mode")
        return None


def run_test(args):
    pad = DualSense()
    if not pad.connect():
        sys.exit("No DualSense found.")
    steps = [
        ("L2: brake ramp (press L2 slowly)", dict(l2=trig_feedback(BRAKE_CURVE), rgb=(0, 0, 255))),
        ("L2: wheel-lock judder (hold L2)", dict(l2=trig_vibration(flat(2, 1), 22), rgb=(255, 0, 0))),
        ("Rumble: big motor", dict(rumble_low=200)),
        ("Rumble: small motor", dict(rumble_high=200)),
        ("Player LEDs rev bar", None),
    ]
    try:
        for name, kw in steps:
            print(name)
            end = time.time() + 3
            while time.time() < end:
                if kw is None:
                    n = int((3 - (end - time.time())) * 2) % 6
                    pad.send(player_leds={0: 0, 1: 1, 2: 3, 3: 7, 4: 15, 5: 31}[n], rgb=(0, 200, 0))
                else:
                    pad.send(**kw)
                time.sleep(0.05)
    finally:
        pad.close()
    print("test done")


def main():
    p = argparse.ArgumentParser(description="DiRT Rally 2.0 -> DualSense adaptive triggers & rumble")
    p.add_argument("--test", action="store_true", help="cycle effects on the controller without the game")
    p.add_argument("--ip", default="127.0.0.1")
    p.add_argument("--port", type=int, default=20777)
    p.add_argument("--rumble", type=float, default=1.0, help="rumble strength multiplier (0 = off)")
    p.add_argument("--triggers", type=float, default=1.0, help="trigger strength multiplier (0 = off)")
    p.add_argument("--spin-threshold", type=float, default=0.15, help="wheelspin sensitivity (lower = earlier)")
    p.add_argument("--lock-threshold", type=float, default=0.25, help="brake-lock sensitivity (lower = earlier)")
    p.add_argument("--no-lights", action="store_true", help="leave lightbar / player LEDs alone")
    p.add_argument("--gas-blip", type=int, default=3, choices=range(1, 9), metavar="1-8",
                   help="strength of the brief gas-trigger push on each gear change")
    p.add_argument("--no-haptics", action="store_true", help="use old rumble-motor mode instead of HD haptics")
    p.add_argument("--haptics-gain", type=float, default=4.0, help="overall HD haptics volume")
    p.add_argument("--timeout", type=float, default=1.0, help="seconds without data before releasing triggers")
    p.add_argument("-v", "--verbose", action="store_true", help="print live telemetry")
    p.add_argument("--record", action="store_true", help="save raw telemetry to recordings\\ for tuning")
    args = p.parse_args()
    if args.test:
        run_test(args)
    else:
        run_bridge(args)


if __name__ == "__main__":
    main()
