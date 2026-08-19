# testCell — install, wire, and run

A single Python file on the Pi 5. HDMI monitor, USB keyboard and mouse, and an
emergency-off switch that removes motor power without involving the Pi at all.

---

# Part 1 — Emergency off

**Read this before wiring anything else.** The emergency-off switch is not
connected to the Pi. Not to a GPIO, not to a sense line, not to anything this
program can see. It sits in the motor power path and removes power when
pressed — whether this program is running, hung, crashed, or the Pi is
unplugged.

That is the entire design goal: the stop does not depend on software.

## Option A — switch in the battery lead (small stands only)

The simplest possible arrangement, and only viable at low current.

```
Battery + ──[ E-STOP, NC ]── ESC V+
Battery − ─────────────────── ESC V−
```

The switch must be **DC-rated** at or above the motor's stall current. This is
the catch: AC switches quench their arc when the current crosses zero, and DC
never does. A 40 A motor needs a genuine DC-rated emergency-stop switch, which
is expensive and physically large. Below about 10 A this is practical; above
it, use Option B.

## Option B — switch drives a contactor (recommended)

A small switch controls a big contactor. Still no Pi involvement.

```
                    ┌── [ E-STOP, NC ] ──┐
   12 V coil supply ┤                    ├── contactor coil ── coil −
                    └────────────────────┘

Battery + ──[ pre-charge 10R + relay ]──┬── [ CONTACTOR ]──┬── ESC V+
                                        └──────────────────┘
Battery − ────────────────────────────────────────────────── ESC V−
```

- **Normally closed.** Pressing the button, cutting a wire, or unplugging a
  connector all open the coil circuit and drop the contactor. A failure in the
  safety wiring stops the test rather than quietly disabling the protection.
- **DC-rated contactor**, sized for stall current, not nominal draw.
- **Pre-charge**, 10 Ω 10 W with a small relay closing ~500 ms before the main
  contactor. Closing onto discharged ESC bus capacitors draws an inrush spike
  that pits and eventually welds the contacts — a welded contactor is an
  emergency stop that silently no longer works.
- **Bleed resistor**, 10 kΩ 1 W across the ESC input, so the bus caps drain
  after the stop.
- **Latching mushroom head**, so a released button does not restore power on
  its own.

## How the software finds out

There is no sense wire and no voltage sensor, so the only evidence available is
current: if throttle is commanded for a full second and no current flows, the
motor has no power. That latches `motor_power_removed` and drops the commanded
throttle to idle.

That is bookkeeping, not protection. The protection already happened in the
wiring. Its purpose is that the display does not disagree with reality, and
that the throttle is at idle when power comes back.

**Know the limit of this inference.** It only works once you ask for throttle.
An idle ESC draws almost nothing either way, so a stand whose emergency-off is
pressed reads `MOTOR POWER: UNKNOWN`, not `REMOVED`, until you command
throttle. Arming can no longer be blocked on power being present, because there
is nothing to check. Confirm the switch by hand in the pre-run checklist
instead of expecting the screen to tell you.

---

# Part 2 — Pin connections

The Pi drives the ESC and reads both chips directly. No microcontroller.

## HX711 (load cell amplifier, green MH board)

Two headers. Load cell on one edge, Pi on the other.

**Load cell → HX711**

| Load cell wire | HX711 pin | Function |
|---|---|---|
| Red | `E+` | bridge excitation + |
| Black | `E−` | bridge excitation − |
| Green | `A+` | signal + |
| White | `A−` | signal − |
| Shield / braid | chassis ground, **one end only** | |

Use channel A (`A+`/`A−`), gain 128. Leave `B+`/`B−` unconnected — that channel
is gain 32, for a second coarser sensor.

**HX711 → Pi**

| HX711 pin | Pi BCM | Pi header pin | Note |
|---|---|---|---|
| `VCC` | 3V3 | **1** | 3.3 V, not 5 V |
| `GND` | GND | **6** | |
| `DT` | GPIO 5 | **29** | data out |
| `SCK` | GPIO 6 | **31** | clock |

Two things about this board:

- **Power it from 3.3 V.** At 5 V it drives `DT` at 5 V into a 3.3 V GPIO. The
  bridge is ratiometric, so 3.3 V costs signal amplitude you do not need — a
  2 mV/V cell still reads about 6 mV at full 20 kg, an order of magnitude above
  the noise floor.
- **It is hardwired to 10 SPS.** The RATE pin (chip pin 15) is tied to ground.
  10 SPS aliases prop dynamics badly — you get a smooth curve that hides real
  oscillation. For 80 SPS, lift chip pin 15 off its pad and bridge it to pin 16
  (DVDD) right beside it. Do this before mounting anything.

## ADS1115 (16-bit ADC, blue board)

| ADS1115 pin | Connects to | Pi header pin |
|---|---|---|
| `VDD` | 3V3 | **17** |
| `GND` | GND | **9** |
| `SDA` | GPIO 2 / SDA1 | **3** |
| `SCL` | GPIO 3 / SCL1 | **5** |
| `ADDR` | GND | — (sets address `0x48`) |
| `ALRT` | not connected | — |
| `A0` | ACS758 `OU2` (filtered) via 10 kΩ / 10 kΩ divider | — |
| `A1` | ACS758 `OU1` (raw) via 10 kΩ / 10 kΩ divider | — |
| `A2`, `A3` | spare | — |

Put 100 nF from each used input to ground. Use 1 % metal film for the dividers;
their tolerance lands directly in your readings.

The ADS1115 runs at 3.3 V, so its absolute input maximum is 3.6 V. The ACS758
swings to 5 V under fault, so **both** outputs go through their own 10 kΩ /
10 kΩ divider. Use the same resistors on both — a mismatch shows up as a fake
ripple reading that never goes away.

At PGA ±2.048 V (62.5 µV per LSB): 10 mV/A halved by the divider is 5 mV/A at
the pin, so **12.5 mA per LSB**, sitting at 1.25 V for zero current and 1.45 V
at 40 A.

**There is no voltage sensor.** Power is estimated as `nominal volts × amps`
and written to the CSV as `watts_est`. Set the nominal with `--nominal-volts`.
Packs sag under load, so this overstates power at high throttle — good enough
for thrust-vs-throttle curves, not good enough to compare props on g/W. If you
later want real efficiency numbers, add a 47 kΩ / 3.3 kΩ divider from the bus
to `A2` and measure it; the spare channel is there for exactly that.

## ACS758ECB-200B (current sensor) — both outputs used

| ACS758 pin | Connects to |
|---|---|
| `IP+` | battery side of the feed, **bolted lug** |
| `IP−` | ESC side of the feed, **bolted lug** |
| `VCC` | 5 V (Pi header pin 2 or 4) |
| `GND` | GND (Pi header pin 39) |
| `OU2` | 10 k / 10 k divider → ADS1115 `A0` — filtered |
| `OU1` | 10 k / 10 k divider → ADS1115 `A1` — raw |

`OU1` and `OU2` are two taps on the same measurement: `OU1` is the chip's
output, `OU2` is that signal through the board's RC filter. Reading both costs
one spare ADC channel and buys three things:

- `OU2` gives a low-noise current for logging;
- `OU1` keeps the transients `OU2` smooths away, so inrush and commutation
  spikes stay visible instead of being averaged out of existence;
- their difference is a live ripple figure, shown on screen and logged as
  `ripple_a`. Ripple that climbs with throttle is normal. Ripple that appears
  suddenly is usually a failing capacitor or a loose lug.

Both are zeroed independently at INIT CELL. They do not share an offset,
because the filter sits after it. At rest they must agree within about 50 mV;
the self-check fails if they do not, which is how a swapped `OU1`/`OU2` or a
wrong divider resistor gets caught before a run.

`IP+`/`IP−` carry full motor current through the package's internal busbar.
Bolted lugs only — never header pins, never PCB traces at these widths. The
sensing side is galvanically isolated from that path, which is why this sensor
can sit in the hot leg with no ground concerns.

> The ±200 A part is heavily oversized for a prop stand. Its total error is
> about ±1.2 % of full scale — roughly **±2.4 A regardless of what you draw**.
> At a 10 A operating point that's a 24 % error on the current term, and it
> propagates into your g/W figure. Fine for thrust curves; not for comparing
> props on efficiency. An ACS758ECB-**050B** (40 mV/A, same footprint and
> wiring) quarters the error.

## ESC signal

| Signal | Pi BCM | Pi header pin |
|---|---|---|
| ESC signal | GPIO 12 | **32** |
| ESC ground | GND | **34** |

Leave the ESC's red BEC wire disconnected. Pi and ESC must share a ground or
the pulse has no reference.

**Hardware PWM is required.** Software PWM on Linux jitters by tens of
microseconds because the kernel can preempt the toggling thread, and on a
1000–2000 µs servo pulse that jitter is throttle noise. Enable the peripheral:

```bash
sudo nano /boot/firmware/config.txt
```

Add, then reboot:

```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

Verify:

```bash
ls /sys/class/pwm/          # a pwmchip should be listed
```

---

# Part 3 — Install

```bash
sudo apt update
sudo apt install -y python3-venv python3-matplotlib python3-lgpio \
                    python3-smbus2 i2c-tools
```

Enable I2C:

```bash
sudo raspi-config      # Interface Options -> I2C -> Yes
sudo reboot
```

Confirm the ADS1115 answers:

```bash
i2cdetect -y 1
```

You should see `48` in the grid. Nothing there means wiring, power, or a
missing `ADDR`-to-GND link.

## Virtual environment

```bash
mkdir -p ~/TestCell_pi && cd ~/TestCell_pi
# copy testCell.py and testCell_cells.json here
python3 -m venv --system-site-packages testCell
source testCell/bin/activate
python -c "import matplotlib, lgpio, smbus2; print('ready')"
```

`--system-site-packages` is deliberate. `lgpio` is a compiled package installed
by apt; an isolated venv cannot see it, and pip-installing it means building
from source with swig and `python3-dev`.

**SIM mode needs none of the hardware libraries** — only matplotlib for the
display. You can run the simulator on a bare Pi, or on your laptop, before any
hardware exists.

## Permissions

Your user needs group membership for GPIO, I2C, and the PWM sysfs interface:

```bash
sudo usermod -aG gpio,i2c,dialout $USER
```

Log out and back in. If hardware PWM still gives permission errors, add a udev
rule:

```bash
sudo tee /etc/udev/rules.d/99-pwm.rules >/dev/null <<'EOF'
SUBSYSTEM=="pwm", ACTION=="add", RUN+="/bin/chgrp -R gpio /sys%p", RUN+="/bin/chmod -R g=u /sys%p"
EOF
sudo udevadm control --reload
```

---

# Part 4 — Running in SIM

Always start here. SIM synthesizes every input — thrust, volts, amps, PWM, and
the power path — and holds no serial handle, no GPIO handle, and no PWM handle.
There is no code path from it to a motor.

```bash
cd ~/TestCell_pi
source testCell/bin/activate
python testCell.py --screen 1920x1080
```

The banner reads `SIMULATION - NO MOTOR - DATA IS SYNTHETIC` in orange.

Work through the sequence, by button or key:

| Step | Key | What happens |
|---|---|---|
| 1 | `e` | **INIT ENV** — backend up, self-check |
| 2 | `c` | **INIT CELL** — sensors zeroed |
| 3 | `a` | **ARM** — one press; also clears the startup latch |
| 4 | `r` | **RUN** — the throttle profile executes |

Then rehearse the stop. Press `f` (**SIM FAULT**) mid-run: it simulates the
emergency-off by collapsing the simulated bus. Confirm the console latches
`motor_power_removed`, the indicator turns red, and neither CLEAR nor ARM works
until you press `f` again to restore power.

**Do that rehearsal before every real session.** It is how you learn what the
screen looks like when things go wrong, at a time when nothing can be damaged.

Over SSH or the Pi Connect shell, add `--headless` for a terminal interface
with the same commands.

## The default view

The console shows thrust and current only — one wide plot each, plus large
THRUST and AMPS readouts. This is the default (`--plots 2`), not a
stripped-down mode; it is what most sessions actually need. Estimated power
and the raw/filtered current comparison are still written to the CSV
regardless — nothing is lost, only hidden from the screen. Add `--plots 4`
if you want them on screen too.

## If the console feels slow

The interface only redraws the plot traces and the numbers every frame — the
buttons, slider, and text boxes stay as they were until something about them
actually changes, at which point a full redraw happens once. A full redraw of
the whole window costs roughly 100 ms on a desktop and several times that on a
Pi, which is the usual cause of a sluggish window.

Check these first, before changing any settings:

```bash
vcgencmd get_throttled     # anything but throttled=0x0 means undervoltage
vcgencmd measure_temp      # sustained 80C+ means thermal throttling
top                        # is python actually using a whole core?
```

`get_throttled` is the one that catches people out. A Pi 5 on an undersized
USB-C supply throttles hard, and the symptom is a slow interface rather than
an error message. Use a genuine 5 V 5 A supply.

**Are you looking at the screen over Pi Connect or VNC rather than the wired
monitor?** Remote desktop re-encodes every changed pixel over the network, so
a plot that redraws fine locally can crawl remotely. That is a property of the
remote session, not the program — try `--headless` as a quick check: if the
terminal interface feels instant while the graphical one does not, the cost is
in getting pixels to that particular screen, not in the program's own work.
The wired monitor is the normal operating path for exactly this reason.

If it is still slow on the local wired display, trade detail for speed:

```bash
python testCell.py --fps 5 --plot-points 120
```

| Flag | Effect |
|---|---|
| `--plots 4` | Also shows power and the raw/filtered current comparison. More to draw, so only add it if you want it. |
| `--fps 5` | Slower refresh. The sampling rate itself is unaffected. |
| `--plot-points 120` | Shorter trace history, less to draw per frame. |
| `--headless` | Terminal interface. Costs almost nothing to draw. |
| `--no-blit` | Full redraws every frame. Much slower; only for diagnosing display artefacts. |

None of these change how fast the sensors are read or what lands in the CSV.
They only change how much of the window gets drawn.

---

# Part 5 — Running LIVE

Only after Parts 1 and 2 are wired and checked.

## Pre-run checklist

1. Prop guarded. Stand bolted down. Everyone behind the blast line.
2. Load cell calibrated (below). Recalibrate after any teardown.
3. **Press the emergency-off and confirm the motor cannot be powered.** With
   the console open, the indicator must read `MOTOR POWER: REMOVED` in red.
   Release it and confirm it returns to green. Do this every session — it is
   the only proof the switch and contactor still work.
4. Verify the throttle is at idle and the ESC is disarmed.

## Start

```bash
python testCell.py --mode live --screen 1920x1080
```

The banner turns red: `LIVE - MOTOR CAN SPIN`. There is no startup flag
gating this — the physical emergency-off in the motor power path is what
actually stops the motor, not a software permission check. Be at the stand,
hand near the switch, before commanding LIVE throttle.

Both ARM and LIVE are single presses — neither needs confirming twice. ARM
is gated by the phase order (env and cell must be initialized first) and by
the emergency-off; LIVE has no gate at all beyond disarming first if a
profile is running.

Same sequence: `e` → `c` → `a` → `r`. Throttle by hand with `+`/`−`
or the slider, or run the stepped profile.

To take telemetry from a microcontroller instead of wiring the chips to the Pi:

```bash
python testCell.py --mode live --backend serial
```

## Calibration

**Load cell, two points:**

1. Stand assembled, prop off, nothing touching the cell. Press `c` to zero.
2. Hang a known mass along the thrust axis — 1 kg and 2 kg calibration weights,
   or anything verified on a scale.
3. `LOADCELL_SCALE = counts / grams`. Set it at the top of `testCell.py`.
4. Check with a second mass. If it is off by more than a percent, the cell is
   seeing off-axis load — fix the mount, not the number. Bending moments from
   an off-axis mount are the most common reason a stand reads differently after
   every teardown.

**Current:** the software zeros both outputs on every INIT CELL, so offset is
handled. For span, put a clamp meter on the DC feed at two steady throttle
settings and compare against `amps_filt`. A consistent proportional error means
`ACS_MV_PER_AMP` or a divider ratio is off — measure the actual resistors and
use those values.

**Nominal voltage:** meter the pack at rest and pass it with
`--nominal-volts`. This is an assumption, not a calibration; `watts_est` is
only as good as that number, and gets worse as the pack sags.

---

# Part 6 — Data

```
TestCell_files/<label>/YYYY-MM-DD-HH-MM-SS.csv
TestCell_files/<label>/..._meta.txt
```

Columns: `host_iso, host_s, dev_ms, thrust_g, thrust_lb, amps_filt, amps_raw,
ripple_a, volts_nom, watts_est, pwm, mode`.

Thrust is logged in grams and pounds. On screen the large number is grams,
with kilograms and pounds underneath it, and the thrust plot carries a pounds
scale on its right edge.

`amps_filt` is `OU2`, `amps_raw` is `OU1`, and `watts_est` is an estimate from
`volts_nom` — the name is deliberate, so a column of estimated power is never
mistaken for a measurement.

SIM runs get a `SIM_` filename prefix, `mode=sim` on every row, and a warning
in the sidecar, so simulated data can never be mistaken for measurements.

Fill in ambient conditions and notes in the sidecar before you leave the lab.
Six months from now the filename will not tell you what was on the stand.

---

# Troubleshooting

| Symptom | Cause |
|---|---|
| `no hardware PWM channel` | `dtoverlay=pwm-2chan` missing from config.txt, or no reboot since adding it. |
| `HX711 not responding` | Check 3.3 V and ground first, then DT on pin 29 and SCK on pin 31. Swapped DT/SCK is the usual culprit. |
| `ADS1115 not responding` | `i2cdetect -y 1` should show `48`. Nothing means wiring, or `ADDR` not tied to GND. |
| Thrust reads but drifts wildly | Load cell cable running parallel to the phase wires. Route it perpendicular, never in the same bundle. |
| Current reads ~2 A with nothing running | The ±200 A part's offset. INIT CELL re-zeros it; if it persists, the divider resistors are off. |
| `MOTOR POWER: UNKNOWN` after arming | Normal. It stays unknown until you command throttle — there is nothing to measure at idle. |
| Latches `motor_power_removed` with power on | No current is reaching the sensor. Check the `IP+`/`IP−` lugs, the contactor, and the fuse. |
| OU1 and OU2 read far apart at rest | No longer flagged automatically, but still worth checking by hand: swapped channels, mismatched divider resistors, or a loose connection on one path. Both dividers must be the same pair. |
| Ripple reads high and constant | Mismatched dividers on `OU1` and `OU2`, not a real electrical problem. |
| Permission denied on GPIO or PWM | `sudo usermod -aG gpio,i2c $USER`, then log out and back in. |
