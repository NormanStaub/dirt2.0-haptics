# DiRT Rally 2.0 → DualSense haptics & adaptive triggers

DiRT Rally 2.0 has no native DualSense support on PC. This bridge reads the game's
UDP telemetry and drives the PS5 controller's **HD haptics** (the voice-coil
actuators, the same way PS5 games do it) and **adaptive triggers**. No paid tools.

## What you feel

**HD haptics (USB):** layered so each sensation means something specific:

| Layer | When | Feel |
|---|---|---|
| Speed | always (background) | faint, steady rolling; pitch rises with speed, level barely changes |
| Surface | tarmac vs gravel | tarmac smooth and quiet; gravel coarse grain + scattered stone ticks |
| Slide | slip angle > ~10°, full at ~35° | heavy, low, wallowing drag with irregular "grabs"; road texture fades back, so losing grip feels like losing touch with the road |
| Wheelspin | wheels 20 %+ faster than the car | thin, fast, fluttering buzz |
| Lock-up | wheels stopped under braking | road drops out, a skipping skid takes over |
| Impacts | big hits / landings only | dull thud on the side that took it |
| Drivetrain | near the redline | swells as a shift cue, ragged on the limiter, clunk on shifts |

All layers are textured noise with slow random wander (no beeps or sine tones), so it
reads as the car rather than as effects. Thresholds were calibrated on a recorded stage.

**Adaptive triggers:**
- **R2 (throttle):** free, with a small push on each gear change.
- **L2 (brake):** free, then the lightest resistance in the bottom quarter; faint judder when the wheels lock.

**Lightbar / player LEDs:** shift light and rev bar.

Over Bluetooth, HD haptics aren't available, so it falls back to rumble-motor mode.

## Setup

1. Python 3.10+, then:
   ```
   pip install -r requirements.txt
   ```
2. Turn on DiRT's telemetry. Edit
   `Documents\My Games\DiRT Rally 2.0\hardwaresettings\hardware_settings_config.xml`
   (and `hardware_settings_config_vr.xml` if you play in VR) so the `udp` line reads:
   ```xml
   <udp enabled="true" extradata="3" ip="127.0.0.1" port="20777" delay="1" />
   ```
3. Plug the DualSense in with a **USB cable** (needed for HD haptics).

## Run

- `start.bat`: run the bridge, then launch the game however you like.
- `launch.ps1`: start the bridge and the game together, and stop the bridge when the game closes.
  - Desktop shortcut target:
    `powershell -WindowStyle Hidden -ExecutionPolicy Bypass -File "<path>\launch.ps1"`
  - Or as Steam launch options (Properties → General) so Play in Steam starts it too:
    `powershell -WindowStyle Hidden -ExecutionPolicy Bypass -File "<path>\launch.ps1" %command%`
- `python dirt_dualsense.py --test`: cycle the trigger effects without the game.

### Options

| Option | Default | |
|---|---|---|
| `--haptics-gain` | 4.0 | overall haptics strength |
| `--gas-blip` | 3 | strength (1–8) of the R2 push on gear changes |
| `--triggers` | 1.0 | trigger effect multiplier |
| `--rumble` | 1.0 | haptics/rumble layer multiplier (0 = off) |
| `--no-haptics` | | use rumble-motor mode instead of HD haptics |
| `--no-lights` | | leave the lightbar and player LEDs alone |
| `--record` | | save raw telemetry to `recordings\` for tuning |
| `-v` | | print live telemetry |

## Tuning with recordings

```
start.bat --record                         # drive a stage
python replay.py                           # replay the newest recording into the running bridge
python replay.py --start 60 --len 90       # just a section
```
Sit in the game's menus while replaying so live data doesn't mix in. Recordings are
git-ignored.

## How it works

- **Telemetry:** Codemasters "extradata=3" UDP packet (66 little-endian floats): speed,
  velocity and body axes, suspension position/velocity, wheel speeds, inputs, gear, RPM.
- **Haptics:** over USB the DualSense is a 4-channel audio device. Channels 3–4 go
  straight to the left/right actuators. `haptics.py` synthesizes that signal in real time
  (`sounddevice`, WASAPI, ~5 ms blocks).
- **Triggers / lights:** HID output report `0x02` (USB) or `0x31` with CRC32 (Bluetooth),
  via `hidapi`. Trigger effects use the feedback (`0x21`) zone format.

## Files

| File | |
|---|---|
| `dirt_dualsense.py` | bridge: telemetry → triggers, lights, haptics parameters |
| `haptics.py` | real-time HD haptics synthesizer |
| `replay.py` | replay recorded telemetry into the bridge |
| `launch.ps1` | start bridge + game together, stop bridge on exit |
| `start.bat` | run the bridge in a console window |
