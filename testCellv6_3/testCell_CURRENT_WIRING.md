# testCell — current sensing wiring guide

Covers the current measurement path only: ACS758 hall sensor → dividers →
ADS1115 → Pi. For the load cell, see the HX711 wiring reference; for
everything else, see `testCell_SETUP.md`.

---

## Why two channels

The ACS758 breakout brings out **two outputs on the same measurement**:

- **OU1** — the chip's raw output
- **OU2** — that same signal through the board's onboard RC filter

Both get wired to the ADS1115, on separate channels. This isn't redundancy —
it buys three things:

- **OU2 (filtered)** is the current you log and use for every calculation.
- **OU1 (raw)** keeps the fast transients OU2's filter smooths away, so
  inrush and commutation spikes stay visible instead of being averaged out.
- **Their difference is a live ripple figure.** Ripple that grows with
  throttle is normal. Ripple that appears suddenly usually means a failing
  capacitor or a loose connection — this is also your main diagnostic for
  a bad wiring joint, since both channels read the same physical current
  and must agree at rest.

---

## ACS758 — the high-current side

| ACS758 pin | Connects to |
|---|---|
| `IP+` | battery side of the motor feed — **bolted lug** |
| `IP−` | ESC side of the motor feed — **bolted lug** |

These two tabs carry your **full motor current** through the chip's internal
busbar. Bolted connections only — never header pins, never a breadboard,
never thin jumper wire. This is a fire risk under load if done wrong, not
just a bad reading.

Polarity determines the sign of your current reading, not whether the sensor
works — the ACS758 is bidirectional. If your logged current comes out
negative once everything is powered, swap `IP+`/`IP−`; that's the fix, not a
wiring fault.

---

## ACS758 — the low-voltage side

| ACS758 pin | Connects to | Pi header pin |
|---|---|---|
| `VCC` | 5V | 2 or 4 |
| `GND` | GND | 39 |
| `OU2` (filtered) | 10kΩ/10kΩ divider → ADS1115 `A0` | — |
| `OU1` (raw) | 10kΩ/10kΩ divider → ADS1115 `A1` | — |

**Both outputs need their own divider**, and **both dividers must use the
same resistor pair.** The ADS1115 runs at 3.3V (absolute max input 3.6V),
but the ACS758 output swings up to 5V under fault — hence the divider on
both channels, not just one. A mismatched pair between the two dividers
shows up as a fake ripple reading that never goes away, since a divider
mismatch alone is enough to make the two channels disagree — even with a
perfectly healthy sensor and zero current flowing.

```
ACS758 OU2 ──┬── 10kΩ ──┬── ADS1115 A0
             │          │
           (from        10kΩ
            OU2)         │
                         GND

ACS758 OU1 ──┬── 10kΩ ──┬── ADS1115 A1
             │          │
           (from        10kΩ
            OU1)         │
                         GND
```

Add 100nF from each ADS1115 input to ground. Use 1% metal film resistors —
their tolerance lands directly in your current readings.

---

## ADS1115 → Pi

| ADS1115 pin | Connects to | Pi header pin |
|---|---|---|
| `VDD` | 3V3 | 17 |
| `GND` | GND | 9 |
| `SDA` | GPIO 2 / SDA1 | 3 |
| `SCL` | GPIO 3 / SCL1 | 5 |
| `ADDR` | GND (sets address `0x48`) | — |
| `A0` | ACS758 OU2, via divider | — |
| `A1` | ACS758 OU1, via divider | — |
| `A2`, `A3` | spare | — |
| `ALRT` | not connected | — |

---

## What the numbers should look like

At rest (no current flowing), **OU1 and OU2 should agree within ~50 mV** at
the ADS1115 pins — they're two taps on the same physical current, so a
bigger gap than that means a wiring or resistor problem, not calibration
drift. The software reports both values independently at INIT ENV rather
than comparing them for you, so checking this is on you: read both numbers
off the self-check output and compare by eye.

At PGA ±2.048V (62.5µV/LSB), with the 10k/10k divider halving the ACS758's
10mV/A output to 5mV/A at the pin:

| Condition | Voltage at ADS1115 pin | Current |
|---|---|---|
| 0 A | ~1.25 V | 0 A |
| 40 A | ~1.45 V | 40 A |

**Resolution:** 12.5 mA per LSB.

**Zeroing:** the software measures each channel's actual zero-current
output on every INIT CELL (with the contactor open, guaranteed by the
phase sequence) — this removes the sensor's offset error, which is the
largest single error term on this part. Always run INIT CELL before
trusting a current reading.

---

## Quick fault check

| Symptom | Likely cause |
|---|---|
| OU1 and OU2 read far apart at rest | Not flagged automatically by the software — check by hand. Swapped channels, mismatched divider resistors, or a loose connection on one path |
| Current reads negative | `IP+`/`IP−` swapped — not a fault, just flip them |
| Ripple reads high and constant | Mismatched divider resistors, not an electrical problem on the stand |
| Current reads ~2A with nothing running | The ACS758's offset error, not yet zeroed — run INIT CELL |
| No I2C response at `0x48` | Check `ADDR` is tied to GND, and `SDA`/`SCL` aren't swapped |
