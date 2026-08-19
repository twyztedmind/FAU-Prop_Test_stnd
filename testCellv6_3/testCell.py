#!/usr/bin/env python3
"""
testCell.py -- self-contained propeller thrust stand controller for the Pi 5.

One file. No PC, no network client, no second process, no simulation mode.
Run it on the Pi -- over an attached monitor or SSH -- and it gives you the
whole stand: safety state machine, live sensor readout, and CSV logging.

    python3 testCell.py                 # desktop console
    python3 testCell.py --headless      # terminal interface, for SSH

NORMAL OPERATION: the Pi is physically mounted at the stand, wired to the
sensors over USB/GPIO, with an HDMI monitor, USB keyboard, and mouse permanently
attached -- there is no remote step in ordinary use. Every control has both a
button and a key. Over SSH it falls back to a terminal interface.

CONNECTING TO HARDWARE
-----------------------
There is no manual "Init Env" step. On launch, and automatically again after
any hardware fault, this program keeps trying to open the sensors and the PWM
output on its own, in the background, until it succeeds -- the banner reads
CONNECTING TO HARDWARE while that is happening. Once connected, live thrust
and current readings appear immediately and keep updating continuously,
whether or not the stand is armed. Bench-checking a sensor -- is the load
cell wired right, does the number move when you press on it -- is a normal
thing to want to do, and does not require arming.

ARM is the only gate before commanding throttle or running a profile. There
is no separate "cell" phase to complete first.

TARE zeros the thrust reading and re-zeros the current sensor's offset. It is
available any time the stand is connected and not mid-profile -- press it
whenever you want the displayed thrust to read 0, not as a one-time setup
step.

EMERGENCY OFF
-------------
The emergency-off switch is wired directly into the motor power path. It is not
connected to the Pi at all -- not a GPIO, not a sense line, nothing. Pressing it
removes power from the ESC whether this program is running, hung, crashed, or
the Pi is unplugged. That is the whole point: the stop does not depend on
software, on GPIO, or on the Pi existing.

Because there is no sense wire and no bus-voltage sensor, this program infers
the stop from current: if throttle is commanded for a second and no current
flows, the motor has no power, and a stop is latched. That is bookkeeping, not
protection -- the protection already happened in the wiring. Note the limit of
the inference: it can only notice once you ask for throttle, so a stand whose
emergency-off is pressed reads "unknown" rather than "removed" until you try.

CURRENT
-------
Both hall-sensor outputs are read. OU2 (filtered) is the logged current; OU1
(raw) keeps the transients OU2 smooths away; their difference is a live ripple
figure. There is no voltage sensor, so power is estimated from a nominal pack
voltage and labelled est_W -- packs sag under load, so it overstates at high
throttle. Good for thrust curves, not for comparing props on g/W.

PHASES
------
    disconnected -> ready -> armed -> running

"disconnected" is automatic and self-healing: the program returns to it on any
hardware fault (I2C loss, a serial disconnect) and keeps retrying in the
background with no operator action needed. "ready" means telemetry is live.
ARM is the only step that requires a deliberate action.

SAFETY MODEL
------------
  Layer 0  The physical emergency-off switch removes power from the ESC
           directly, in the motor power path. It works with the Pi unplugged
           or this process dead. Nothing in this file can defeat it, sense
           it, or is required for it to work -- see EMERGENCY OFF above.
  Layer 1  Software: the on-screen E-STOP, the UI-liveness watchdog (stops the
           motor if the interface itself stalls), serial/I2C loss, and any
           unhandled exception all latch a stop.

This program has no way to tell whether the person running it is standing at
the stand or connected over SSH from somewhere else -- unlike the emergency-off
switch, that is not something software can verify. The physical switch is the
backstop either way, but be at the stand, hand near it, before commanding
throttle. A watchdog that reacts after something has already gone wrong is not
a substitute for reaching the switch.
"""

import argparse
import csv
import logging
import logging.handlers
import os
import platform
import queue
import select
import signal
import sys
import threading
import time
import traceback
from datetime import datetime

# Bump this string whenever the file changes in a way worth knowing about.
# It is printed at startup and written to the first line of every log
# session, specifically so that a pasted log -- or a screenshot of the
# terminal -- says unambiguously which copy of the script produced it. Most
# of the confusion in getting this stand running has been about exactly
# that: a fix made to one copy of the file while an older copy kept running
# on the Pi. This ends the guessing.
SCRIPT_VERSION = "2026-08-19.1"

# =========================================================================
# Configuration
# =========================================================================
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

# ---- direct backend: Pi wired straight to the two breakout boards --------
HX_DT_PIN = 5           # BCM 5,  header pin 29   HX711 DT  (DOUT)
HX_SCK_PIN = 6          # BCM 6,  header pin 31   HX711 SCK
I2C_BUS = 1             # BCM 2/3, header pins 3/5   ADS1115 SDA/SCL
ADS_ADDR = 0x48         # ADDR tied to GND
PWM_GPIO = 12           # BCM 12, header pin 32   ESC signal (hardware PWM)

# Load cell: counts per gram. Derive with a known mass, see testCell_SETUP.md.
LOADCELL_SCALE = 420.0

# ---- current, from both hall-sensor outputs -----------------------------
# The ACS758 breakout brings out two taps on the same measurement: OU1 is the
# raw output and OU2 is the same signal through the board's RC filter. Reading
# both costs one spare ADC channel and buys three things:
#   * OU2 gives a low-noise current for logging and efficiency;
#   * OU1 keeps the transients OU2 smooths away, so inrush and commutation
#     spikes are visible instead of averaged out of existence;
#   * their difference is a live ripple figure. A ripple that climbs with
#     throttle is normal; one that appears suddenly is usually a failing
#     capacitor or a loose lug.
# Both are reported independently -- see each backend's selfcheck() -- with
# no automatic pass/fail comparison between them. A large gap is still worth
# checking by hand (see testCell_CURRENT_WIRING.md), but the software will
# not block or degrade anything over it.
ACS_FILT_CH = 0         # ADS1115 A0  <- OU2, filtered
ACS_RAW_CH = 1          # ADS1115 A1  <- OU1, raw
ACS_MV_PER_AMP = 10.0   # ACS758ECB-200B, bidirectional
ACS_DIVIDER = 2.0       # 10k/10k divider ahead of each ADC input

# No bus-voltage sensor. Power is estimated from a nominal pack voltage the
# operator enters, and is labelled est_W everywhere so it is never mistaken for
# a measurement. Real packs sag under load, so this overstates power at high
# throttle -- fine for thrust curves, not for comparing props on g/W.
NOMINAL_VOLTS = 25.2

# Thrust is measured in grams and shown in grams, kilograms and pounds-force
# together. Grams is the raw quantity; kg and lbf are the ones people actually
# compare against a motor's published thrust, and converting in your head at
# the stand is how numbers get written down wrong.
G_PER_KG = 1000.0
G_PER_LB = 453.59237

# Emergency-off detection, without a voltage sensor and without a sense wire.
# If throttle is commanded and no current appears, the motor has no power.
#
# NO_CURRENT_A must sit between the sensor's noise floor and the lowest
# current a genuinely powered motor draws at the lowest commanded throttle --
# not "typical mid-throttle current." Tune with --no-current-amps if a real
# ESC's minimum powered draw differs from this.
NO_CURRENT_A = 0.15     # amps below this counts as "nothing flowing"
NO_CURRENT_S = 1.0      # for this long, while throttle is commanded
THROTTLE_ACTIVE_US = 1050      # IDLE_US + 50; IDLE_US is set below

IDLE_US = 1000
MAX_US = 2000

DATA_ROOT = "TestCell_files"
LOG_DIR = "TestCell_logs"
LOG_FILE = "testCell.log"
LOG_MAX_BYTES = 2_000_000
LOG_BACKUPS = 5

# Firmware command set -- must match TestCell_firmware/testCell_controller.ino
FW_START = b"aa"
FW_STOP = b"xx"
FW_PWM = "p{us}\r"
FW_TARE = b"t\r"

# Stop the motor if the interface stops servicing frames for this long while
# armed. Catches a hung GUI, a wedged remote desktop, or a stalled X server.
UI_WATCHDOG_S = 1.5

# How often to retry opening the hardware after a failed or lost connection.
# Fast enough that plugging in a cable or fixing a loose wire is picked up
# quickly; slow enough not to hammer a bus that just failed.
RECONNECT_S = 3.0

# Cell/run metadata shown on screen and written into each run's sidecar file.
# Purely descriptive -- editable any time, never gates anything. Edit here,
# or drop a testCell_cells.json next to this script with the same shape.
CELL_PRESETS = [
    {"name": "cell-1", "motor": "MN505-S", "prop": "15x5", "supply": "6S 25.2V"},
    {"name": "cell-2", "motor": "MN505-S", "prop": "16x5.4", "supply": "6S 25.2V"},
    {"name": "bench", "motor": "unspecified", "prop": "unspecified",
     "supply": "6S 25.2V"},
]

DEFAULT_PROFILE = [
    (1100, 4), (1200, 4), (1300, 4), (1400, 4), (1500, 4),
    (1600, 4), (1700, 4), (1800, 4), (1900, 4), (2000, 4), (1000, 3),
]

PHASE_DISCONNECTED = "disconnected"
PHASE_READY = "ready"
PHASE_ARMED = "armed"
PHASE_RUNNING = "running"

STOP_STARTUP = "startup_safe_state"
STOP_POWER = "motor_power_removed"
STOP_SOFTWARE = "software_estop"
STOP_SERIAL = "serial_link_lost"
STOP_UI = "interface_stalled"
STOP_SHUTDOWN = "process_shutdown"


# =========================================================================
# Logging
# =========================================================================
LOG = logging.getLogger("testCell")


def setup_logging(log_root=None):
    """Configure LOG and return the log file's path.

    log_root is the directory the log file lives under (default: next to
    this script, so it works the same whether you launched from
    ~/TestCell_pi or anywhere else). Safe to call more than once; later
    calls are no-ops so re-entrant test harnesses do not duplicate handlers.
    """
    if LOG.handlers:
        return LOG.handlers[0].baseFilename

    if log_root is None:
        log_root = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(log_root, LOG_DIR)
    try:
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, LOG_FILE)
        handler = logging.handlers.RotatingFileHandler(
            path, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUPS)
    except OSError as exc:
        logging.basicConfig(level=logging.INFO)
        LOG.warning("could not open log file under %s (%s); logging to "
                    "stderr only for this session", log_dir, exc)
        return None

    handler.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)-7s [%(threadName)s] %(message)s"))
    LOG.addHandler(handler)
    LOG.setLevel(logging.INFO)
    LOG.propagate = False

    LOG.info("=" * 70)
    LOG.info("testCell %s starting  (pid %d)", SCRIPT_VERSION, os.getpid())
    LOG.info("python %s  platform %s", platform.python_version(),
             platform.platform())
    return path


def log_exception(context, exc):
    """Full traceback to the file; a short line is left for self.log()/print
    to show the operator."""
    LOG.error("%s: %s\n%s", context, exc,
              "".join(traceback.format_exception(type(exc), exc,
                                                 exc.__traceback__)))


# =========================================================================
# Backends
# =========================================================================
class SerialBackend:
    """Real hardware, reached through a microcontroller on USB serial."""

    def __init__(self, port=SERIAL_PORT, baud=BAUD, **_kw):
        self.port, self.baud = port, baud
        self.ser = None
        self.buf = b""
        self.bad_lines = 0
        self.good_lines = 0
        self.volts = 0.0

    def open(self):
        import serial
        self.ser = serial.Serial(
            port=self.port, baudrate=self.baud, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
            timeout=0, write_timeout=1.0)
        time.sleep(2.0)
        self._write(FW_STOP)
        self.ser.reset_input_buffer()

    def selfcheck(self):
        msgs, ok = [], True
        if self.ser is None or not self.ser.is_open:
            return False, ["serial port not open"]
        msgs.append(f"serial {self.port} @ {self.baud}")
        self.start_stream()
        deadline, seen = time.time() + 2.0, 0
        while time.time() < deadline and seen < 5:
            seen += len(self.poll())
            time.sleep(0.02)
        self.stop_motor()
        if seen == 0:
            ok = False
            msgs.append("no parseable telemetry: check baud, wiring, firmware")
        else:
            msgs.append(f"telemetry ok, {seen} samples, {self.bad_lines} bad")
        return ok, msgs

    def _write(self, data):
        if self.ser is None:
            return False
        if isinstance(data, str):
            data = data.encode("ascii", "ignore")
        try:
            self.ser.write(data)
            self.ser.flush()
            return True
        except Exception:
            return False

    def start_stream(self):
        return self._write(FW_START)

    def stop_motor(self):
        ok = self._write(FW_PWM.format(us=IDLE_US))
        return self._write(FW_STOP) and ok

    def set_pwm(self, us):
        return self._write(FW_PWM.format(us=int(max(IDLE_US,
                                                    min(MAX_US, us)))))

    def tare(self):
        return self._write(FW_TARE)

    def poll(self, max_lines=40):
        out = []
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.buf += self.ser.read(waiting)
        except (OSError, IOError) as exc:
            raise ConnectionError(f"serial device lost: {exc}") from exc
        if len(self.buf) > 8192:
            self.buf = b""
        while len(out) < max_lines:
            idx = -1
            for sep in (b"\r", b"\n"):
                i = self.buf.find(sep)
                if i != -1 and (idx == -1 or i < idx):
                    idx = i
            if idx == -1:
                break
            raw, self.buf = self.buf[:idx], self.buf[idx + 1:]
            s = self._parse(raw)
            if s is None:
                if raw.strip():
                    self.bad_lines += 1
            else:
                self.good_lines += 1
                out.append(s)
        return out

    @staticmethod
    def _parse(raw):
        try:
            text = raw.decode("ascii", "replace").strip()
        except Exception:
            return None
        if not text:
            return None
        toks = text.split()
        if len(toks) == 11:
            vals = toks[0::2]
        elif len(toks) == 1 and "," in toks[0]:
            vals = toks[0].split(",")
        elif len(toks) == 6:
            vals = toks
        else:
            return None
        if len(vals) != 6:
            return None
        try:
            amps = float(vals[3])
            return {"dev_ms": float(vals[0]), "thrust_g": float(vals[1]),
                    "amps": amps, "amps_raw": amps, "ripple_a": 0.0,
                    "volts_nom": float(vals[2]),
                    "watts_est": float(vals[2]) * amps,
                    "pwm": int(float(vals[5]))}
        except ValueError:
            return None

    def close(self):
        try:
            self.stop_motor()
            time.sleep(0.15)
            if self.ser:
                self.ser.close()
        except Exception:
            pass


class DirectBackend:
    """Pi wired straight to the two breakout boards, no microcontroller.

      HX711   load cell, bit-banged on two GPIOs
      ADS1115 bus volts and amps over I2C
      ESC     servo pulse from the hardware PWM peripheral

    Hardware PWM matters. Software PWM on Linux jitters by tens of
    microseconds because the kernel can preempt the toggling thread. This
    needs `dtoverlay=pwm-2chan` -- see testCell_SETUP.md.

    The HX711 clock line must not stay high for more than 60 us or the chip
    powers down and returns garbage. A preempted bit-bang can exceed that, so
    every read is timed and discarded if the pulse ran long.
    """

    def __init__(self, hx_dt=HX_DT_PIN, hx_sck=HX_SCK_PIN, i2c_bus=I2C_BUS,
                 ads_addr=ADS_ADDR, pwm_gpio=PWM_GPIO,
                 nominal_volts=NOMINAL_VOLTS, **_kw):
        self.nominal_volts = nominal_volts
        self.hx_dt, self.hx_sck = hx_dt, hx_sck
        self.i2c_bus_num, self.ads_addr = i2c_bus, ads_addr
        self.pwm_gpio = pwm_gpio
        self.chip = None
        self.bus = None
        self.pwm_path = None
        self.tare_counts = 0.0
        self.zero_filt = 1.25
        self.zero_raw = 1.25
        self.amps = 0.0
        self.amps_raw = 0.0
        self.thrust = 0.0
        self.bad_lines = 0
        self.good_lines = 0
        self.streaming = False
        self.t0 = time.monotonic()
        self.next_sample = 0.0
        self.cmd_us = IDLE_US

    def _open_chip(self):
        import lgpio
        self.lgpio = lgpio
        last = None
        for n in (0, 4, 1, 2):
            try:
                h = lgpio.gpiochip_open(n)
                info = lgpio.gpio_get_chip_info(h)
                if info[1] >= 30:
                    return h
                lgpio.gpiochip_close(h)
            except Exception as exc:
                last = exc
        raise RuntimeError(f"no usable gpiochip found ({last})")

    def _hx_read_raw(self):
        g = self.lgpio
        if g.gpio_read(self.chip, self.hx_dt):
            return None
        value = 0
        longest = 0.0
        for _ in range(25):
            t = time.perf_counter()
            g.gpio_write(self.chip, self.hx_sck, 1)
            bit = g.gpio_read(self.chip, self.hx_dt)
            g.gpio_write(self.chip, self.hx_sck, 0)
            longest = max(longest, time.perf_counter() - t)
            value = (value << 1) | bit
        if longest > 60e-6:
            self.bad_lines += 1
            return None
        value >>= 1
        if value & 0x800000:
            value -= 0x1000000
        return value

    def _ads_read(self, channel):
        mux = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}[channel]
        cfg = (0x8000 | mux | 0x0400 | 0x0100 | 0x00A0 | 0x0003)
        self.bus.write_i2c_block_data(self.ads_addr, 0x01,
                                      [cfg >> 8, cfg & 0xFF])
        time.sleep(0.0035)
        d = self.bus.read_i2c_block_data(self.ads_addr, 0x00, 2)
        raw = (d[0] << 8) | d[1]
        if raw & 0x8000:
            raw -= 0x10000
        return raw * 2.048 / 32768.0

    def _pwm_open(self):
        import glob
        channel = 0 if self.pwm_gpio in (12, 18) else 1
        for chip in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
            try:
                with open(os.path.join(chip, "npwm")) as fh:
                    if int(fh.read().strip()) <= channel:
                        continue
            except OSError:
                continue
            path = os.path.join(chip, f"pwm{channel}")
            if not os.path.exists(path):
                try:
                    with open(os.path.join(chip, "export"), "w") as fh:
                        fh.write(str(channel))
                    time.sleep(0.1)
                except OSError:
                    continue
            if os.path.exists(path):
                self.pwm_path = path
                self._pwm_write("period", 20_000_000)
                self._pwm_write("duty_cycle", IDLE_US * 1000)
                self._pwm_write("enable", 1)
                return
        raise RuntimeError(
            "no hardware PWM channel. Add 'dtoverlay=pwm-2chan' to "
            "/boot/firmware/config.txt and reboot -- see testCell_SETUP.md")

    def _pwm_write(self, attr, value):
        with open(os.path.join(self.pwm_path, attr), "w") as fh:
            fh.write(str(int(value)))

    def open(self):
        import smbus2
        self.chip = self._open_chip()
        try:
            self.lgpio.gpio_claim_input(self.chip, self.hx_dt)
            self.lgpio.gpio_claim_output(self.chip, self.hx_sck, 0)
        except Exception as exc:
            # "GPIO busy" almost always means a previous instance of this
            # process is still holding the pin -- crashed, killed, or a
            # duplicate still running -- not a wiring problem. Say so: the
            # wiring-focused message elsewhere would send someone checking
            # solder joints for nothing.
            if "busy" in str(exc).lower():
                raise RuntimeError(
                    f"GPIO busy ({exc}). Usually a leftover testCell.py "
                    f"process still holding the pin -- check "
                    f"`ps aux | grep testCell` and kill any stale one, "
                    f"then try again.") from exc
            raise
        self.bus = smbus2.SMBus(self.i2c_bus_num)
        self._pwm_open()
        self.stop_motor()
        time.sleep(0.4)
        self._zero_current()

    def _zero_current(self):
        f = [self._ads_read(ACS_FILT_CH) for _ in range(16)]
        r = [self._ads_read(ACS_RAW_CH) for _ in range(16)]
        self.zero_filt = sum(f) / len(f)
        self.zero_raw = sum(r) / len(r)

    @staticmethod
    def _to_amps(v, zero):
        return (v - zero) * ACS_DIVIDER * 1000.0 / ACS_MV_PER_AMP

    def selfcheck(self):
        msgs, ok = [], True
        msgs.append(f"HX711 on BCM {self.hx_dt}/{self.hx_sck}")
        got = 0
        deadline = time.time() + 2.0
        while time.time() < deadline and got < 3:
            if self._hx_read_raw() is not None:
                got += 1
            time.sleep(0.02)
        if got == 0:
            ok = False
            msgs.append("HX711 not responding: check wiring and 3V3 supply")
        else:
            msgs.append(f"HX711 ok ({self.bad_lines} discarded)")
        try:
            vf = self._ads_read(ACS_FILT_CH)
            vr = self._ads_read(ACS_RAW_CH)
            msgs.append(f"ADS1115 at 0x{self.ads_addr:02x}: "
                        f"OU2 (filtered) {vf:.3f} V")
            msgs.append(f"ADS1115 at 0x{self.ads_addr:02x}: "
                        f"OU1 (raw) {vr:.3f} V")
        except Exception as exc:
            ok = False
            msgs.append(f"ADS1115 not responding: {exc}")
        msgs.append(f"hardware PWM at {self.pwm_path}")
        msgs.append(f"power estimated from {self.nominal_volts:.1f} V nominal "
                    f"(no voltage sensor fitted)")
        return ok, msgs

    def poll(self):
        now = time.monotonic()
        if not self.streaming or now < self.next_sample:
            return []
        self.next_sample = now + 0.02

        raw = self._hx_read_raw()
        if raw is not None:
            self.thrust = (raw - self.tare_counts) / LOADCELL_SCALE
        try:
            self.amps = self._to_amps(self._ads_read(ACS_FILT_CH),
                                      self.zero_filt)
            self.amps_raw = self._to_amps(self._ads_read(ACS_RAW_CH),
                                          self.zero_raw)
        except OSError as exc:
            raise ConnectionError(f"I2C lost: {exc}") from exc

        self.good_lines += 1
        return [{"dev_ms": (now - self.t0) * 1000.0,
                 "thrust_g": round(self.thrust, 1),
                 "amps": round(self.amps, 3),
                 "amps_raw": round(self.amps_raw, 3),
                 "ripple_a": round(abs(self.amps_raw - self.amps), 3),
                 "volts_nom": self.nominal_volts,
                 "watts_est": round(self.nominal_volts * self.amps, 2),
                 "pwm": int(self.cmd_us)}]

    def set_pwm(self, us):
        self.cmd_us = int(max(IDLE_US, min(MAX_US, us)))
        self._pwm_write("duty_cycle", self.cmd_us * 1000)

    def start_stream(self):
        self.streaming = True

    def stop_motor(self):
        self.cmd_us = IDLE_US
        if self.pwm_path:
            try:
                self._pwm_write("duty_cycle", IDLE_US * 1000)
            except OSError:
                pass

    def tare(self):
        vals = []
        deadline = time.time() + 2.0
        while time.time() < deadline and len(vals) < 15:
            r = self._hx_read_raw()
            if r is not None:
                vals.append(r)
            time.sleep(0.01)
        if vals:
            self.tare_counts = sum(vals) / len(vals)
        self._zero_current()

    def close(self):
        """Release everything open() may have claimed.

        Deliberately tolerant of a half-constructed object: _connect() calls
        this after a FAILED open() to release whatever was already claimed
        before the failure, so every step here has to cope with the ones
        after it never having run. Nothing raises -- a cleanup path that can
        itself fail would strand the very resources it exists to free.
        """
        try:
            self.stop_motor()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            if self.pwm_path:
                self._pwm_write("enable", 0)
        except OSError:
            pass
        try:
            if self.bus:
                self.bus.close()
        except Exception:
            pass
        try:
            # self.lgpio only exists once _open_chip() got far enough to
            # import it, which is exactly the case this method cannot assume.
            if self.chip is not None and getattr(self, "lgpio", None):
                self.lgpio.gpio_free(self.chip, self.hx_dt)
                self.lgpio.gpio_free(self.chip, self.hx_sck)
        except Exception:
            pass
        try:
            if self.chip is not None and getattr(self, "lgpio", None):
                self.lgpio.gpiochip_close(self.chip)
        except Exception:
            pass
        self.chip = None
        self.bus = None
        self.pwm_path = None


# =========================================================================
# Recording
# =========================================================================
class Recorder:
    """One CSV per run."""

    HEADER = ["host_iso", "host_s", "dev_ms", "thrust_g", "thrust_lb",
              "amps_filt", "amps_raw", "ripple_a", "volts_nom", "watts_est",
              "pwm"]

    def __init__(self, label, root=DATA_ROOT):
        safe = "".join(c for c in label if c.isalnum() or c in "-_ ").strip()
        self.root = root
        self.label = safe or "unnamed"
        self.dir = os.path.join(root, self.label)
        os.makedirs(self.dir, exist_ok=True)
        self.path = None
        self.fh = None
        self.w = None
        self.rows = 0
        self.t0 = time.monotonic()
        self._n = 0

    def open_for(self, cell):
        self.close()
        stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.path = os.path.join(self.dir, f"{stamp}.csv")
        self.fh = open(self.path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.HEADER)
        self.fh.flush()
        self.rows, self.t0, self._n = 0, time.monotonic(), 0
        with open(os.path.splitext(self.path)[0] + "_meta.txt", "w") as m:
            m.write(f"started: {datetime.now().isoformat()}\n")
            m.write("power: ESTIMATED from nominal voltage, no voltage "
                    "sensor fitted. Packs sag under load, so watts_est "
                    "overstates at high throttle.\n")
            m.write("amps_filt: ACS758 OU2 (filtered)\n")
            m.write("amps_raw:  ACS758 OU1 (unfiltered)\n")
            for k, v in (cell or {}).items():
                m.write(f"{k}: {v}\n")
            m.write("ambient:\nnotes:\n")
        return self.path

    def write(self, s):
        if self.w is None:
            return
        self.w.writerow([datetime.now().isoformat(timespec="milliseconds"),
                         round(time.monotonic() - self.t0, 4), s.get("dev_ms"),
                         s.get("thrust_g"),
                         round(float(s.get("thrust_g") or 0.0) / G_PER_LB, 4),
                         s.get("amps"), s.get("amps_raw"),
                         s.get("ripple_a"), s.get("volts_nom"),
                         s.get("watts_est"), s.get("pwm")])
        self.rows += 1
        self._n += 1
        if self._n >= 20:
            self.fh.flush()
            self._n = 0

    def close(self):
        if self.fh is None:
            return
        try:
            self.fh.flush()
            os.fsync(self.fh.fileno())
            self.fh.close()
        except Exception:
            pass
        self.fh = self.w = None


# =========================================================================
# Controller: safety state machine, on its own thread
# =========================================================================
class Controller:
    """Owns the backend and every decision that can start or stop a motor.

    Runs on a background thread so the interface can never delay a stop. The
    UI only posts commands and reads snapshots.

    Connecting to hardware is automatic, not a command. The thread itself
    tries to open the backend when there is none, and keeps retrying on a
    fixed interval if it fails or is lost -- there is no "Init Env" button;
    if the sensors are not plugged in yet, plug them in and the console picks
    them up within RECONNECT_S with no other action needed.
    """

    def __init__(self, args, recorder):
        self.args = args
        self.rec = recorder
        self.phase = PHASE_DISCONNECTED
        self.backend = None
        self.cell = dict(args.cell)
        self.latched = True
        self.reason = STOP_STARTUP
        self.profile = None
        self.p_idx = 0
        self.p_until = 0.0
        self.running = True
        self.file_open = False
        self.next_connect_attempt = 0.0

        self.cmds = queue.Queue()
        self.samples = queue.Queue(maxsize=4000)
        self.messages = queue.Queue(maxsize=500)
        self.ui_beat = time.monotonic()
        self.power_state = "unknown"
        self.no_current_since = None
        self.ripple = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def post(self, cmd, **kw):
        self.cmds.put((cmd, kw))

    def beat(self):
        """The UI calls this every frame. If it stops, so does the motor."""
        self.ui_beat = time.monotonic()

    def log(self, msg, level=logging.INFO):
        try:
            self.messages.put_nowait(msg)
        except queue.Full:
            pass
        LOG.log(level, msg)

    def snapshot(self):
        return {"phase": self.phase,
                "latched": self.latched, "reason": self.reason,
                "power": self.power_state,
                "ripple": self.ripple,
                "cell": self.cell.get("name", ""),
                "bad": getattr(self.backend, "bad_lines", 0),
                "rows": self.rec.rows,
                "step": self.p_idx,
                "steps": len(self.profile) if self.profile else 0}

    def start(self):
        self.thread.start()

    def _stop_all(self, reason):
        first = not self.latched
        self.latched = True
        self.reason = reason
        self.profile = None
        if self.phase in (PHASE_ARMED, PHASE_RUNNING):
            self.phase = PHASE_READY
        try:
            if self.backend:
                self.backend.stop_motor()
        except Exception as exc:
            log_exception("stop_motor() failed during a stop "
                         f"(reason={reason})", exc)
        if first:
            self.log(f"STOP LATCHED: {reason}", level=logging.WARNING)

    def _teardown(self, reason):
        self._stop_all(reason)
        if self.backend is not None:
            try:
                self.backend.close()
            except Exception as exc:
                log_exception("backend.close() failed during teardown", exc)
        self.backend = None
        self.phase = PHASE_DISCONNECTED
        self.file_open = False

    def _connect(self):
        """Try once to open the backend. Called automatically, not by the
        operator -- see the class docstring."""
        try:
            if self.args.backend == "serial":
                self.backend = SerialBackend(self.args.serial_port,
                                             self.args.baud)
            else:
                self.backend = DirectBackend(
                    hx_dt=self.args.hx_dt, hx_sck=self.args.hx_sck,
                    i2c_bus=self.args.i2c_bus, ads_addr=self.args.ads_addr,
                    pwm_gpio=self.args.pwm_gpio,
                    nominal_volts=self.args.nominal_volts)
            self.backend.open()
        except Exception as exc:
            log_exception("connect: backend.open() failed", exc)
            self.log(f"not connected: {exc}", level=logging.ERROR)
            # Release whatever open() managed to claim before it failed.
            # open() is not atomic: DirectBackend claims the gpiochip and
            # both HX711 pins, then opens I2C, then exports the PWM
            # channel. A failure at any later step (an I2C read error, a
            # missing PWM overlay) leaves the earlier claims held by THIS
            # process. Dropping the reference without closing leaks them,
            # and because the retry loop runs every few seconds, the next
            # attempt then fails with 'GPIO busy' -- caused by the previous
            # failed attempt, not by any other program. That turns one
            # recoverable fault into a permanent one that looks like a
            # stale process. close() is written to tolerate a
            # half-constructed object, so this is safe here.
            try:
                if self.backend is not None:
                    self.backend.close()
            except Exception as close_exc:
                log_exception("cleanup after failed open() also failed",
                             close_exc)
            self.backend = None
            return

        ok, msgs = self.backend.selfcheck()
        for m in msgs:
            self.log(m)
        if not ok:
            self.log("self-check found a problem -- continuing anyway, "
                     "readings may be degraded until this is fixed",
                     level=logging.WARNING)
        self.backend.start_stream()
        self.phase = PHASE_READY
        self.log("connected" if ok else "connected (degraded -- see above)",
                 level=logging.INFO if ok else logging.WARNING)

    def _tare(self):
        """Zero the thrust reading and re-zero the current offset. Available
        any time the stand is connected and not mid-profile -- not a one-time
        setup step."""
        if self.backend is None or self.phase == PHASE_DISCONNECTED:
            self.log("tare refused: not connected")
            return
        if self.phase == PHASE_RUNNING:
            self.log("tare refused: a profile is running")
            return
        self.backend.stop_motor()
        self.backend.tare()
        self.log("thrust zeroed")

    def _clear(self):
        if self.backend is None:
            self.log("clear refused: not connected")
            return
        self.latched = False
        self.reason = ""
        self.log("latch cleared")

    def _arm(self):
        if self.phase == PHASE_DISCONNECTED:
            self.log("arm refused: not connected yet")
            return
        if self.phase in (PHASE_ARMED, PHASE_RUNNING):
            return
        if self.latched:
            if self.reason == STOP_STARTUP:
                self.latched = False
                self.reason = ""
            else:
                self.log(f"arm refused: press CLEAR to acknowledge "
                         f"{self.reason} first")
                return
        self.power_state = "unknown"
        self.no_current_since = None
        self.backend.set_pwm(IDLE_US)
        self.backend.start_stream()
        self.phase = PHASE_ARMED
        self.log("ARMED")

    def _disarm(self):
        self.profile = None
        if self.backend:
            self.backend.stop_motor()
        if self.phase in (PHASE_ARMED, PHASE_RUNNING):
            self.phase = PHASE_READY
        self.log("disarmed")

    def _handle(self, cmd, kw):
        if cmd == "estop":
            self._stop_all(STOP_SOFTWARE)
        elif cmd == "tare":
            self._tare()
        elif cmd == "clear":
            self._clear()
        elif cmd == "arm":
            self._arm()
        elif cmd == "disarm":
            self._disarm()
        elif cmd == "relabel":
            new = kw.get("label")
            if new:
                self.rec = Recorder(new, root=self.rec.root)
                self.file_open = False
                self.log(f"run label: {new}")
        elif cmd == "setcell":
            key, val = kw.get("key"), kw.get("value")
            if key and val:
                self.cell[key] = val
                self.log(f"{key} = {val}")
        elif cmd == "quit":
            self.running = False
        elif self.phase not in (PHASE_ARMED, PHASE_RUNNING) or self.latched:
            self.log(f"{cmd} ignored: not armed")
        elif cmd == "pwm":
            self.profile = None
            self.phase = PHASE_ARMED
            self.backend.set_pwm(kw.get("us", IDLE_US))
        elif cmd == "run":
            self.profile = list(self.args.profile)
            self.p_idx = -1
            self.p_until = 0.0
            self.phase = PHASE_RUNNING
            self.log(f"running profile, {len(self.profile)} steps")
        elif cmd == "abort":
            self.profile = None
            self.phase = PHASE_ARMED
            self.backend.set_pwm(IDLE_US)
            self.log("profile aborted, throttle at idle")

    def _step_profile(self, now):
        if self.profile is None or now < self.p_until:
            return
        self.p_idx += 1
        if self.p_idx >= len(self.profile):
            self.profile = None
            self.phase = PHASE_ARMED
            self.backend.set_pwm(IDLE_US)
            self.log("profile complete, throttle at idle")
            return
        us, secs = self.profile[self.p_idx]
        self.p_until = now + secs
        self.backend.set_pwm(us)

    def _run(self):
        while self.running:
            now = time.monotonic()

            if (self.phase in (PHASE_ARMED, PHASE_RUNNING)
                    and now - self.ui_beat > UI_WATCHDOG_S):
                self._stop_all(STOP_UI)

            if (self.backend is None and self.phase == PHASE_DISCONNECTED
                    and now >= self.next_connect_attempt):
                self.next_connect_attempt = now + RECONNECT_S
                self._connect()

            while True:
                try:
                    cmd, kw = self.cmds.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._handle(cmd, kw)
                except Exception as exc:
                    log_exception(f"command {cmd!r} (args={kw!r}) failed",
                                 exc)
                    self.log(f"command {cmd} failed: {exc}",
                            level=logging.ERROR)
                    self._stop_all("command_exception")

            if self.backend is not None:
                try:
                    for s in self.backend.poll():
                        self._record(s)
                except ConnectionError as exc:
                    self.log(str(exc), level=logging.ERROR)
                    self._teardown(STOP_SERIAL)

            if self.phase == PHASE_RUNNING and not self.latched:
                self._step_profile(now)

            time.sleep(0.004)

    def _check_power(self, s):
        self.ripple = float(s.get("ripple_a") or 0.0)
        if self.phase not in (PHASE_ARMED, PHASE_RUNNING):
            return
        amps = abs(float(s.get("amps") or 0.0))
        if int(s.get("pwm") or IDLE_US) <= THROTTLE_ACTIVE_US:
            self.no_current_since = None
            return
        threshold = getattr(self.args, "no_current_amps", NO_CURRENT_A)
        if amps >= threshold:
            self.power_state = "present"
            self.no_current_since = None
            return
        now = time.monotonic()
        if self.no_current_since is None:
            self.no_current_since = now
        elif now - self.no_current_since > NO_CURRENT_S:
            self.power_state = "removed"
            self._stop_all(STOP_POWER)

    def _record(self, s):
        try:
            self.samples.put_nowait(s)
        except queue.Full:
            pass
        self._check_power(s)

        if self.phase not in (PHASE_ARMED, PHASE_RUNNING):
            return
        if not self.file_open:
            path = self.rec.open_for(self.cell)
            self.file_open = True
            self.log(f"logging to {path}")
        self.rec.write(s)

    def shutdown(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self._teardown(STOP_SHUTDOWN)
        self.rec.close()


# =========================================================================
# Terminal interface
# =========================================================================
class TerminalUI:
    """Single-keystroke console. Works over SSH and the Pi Connect shell."""

    KEYS = """
  a  Arm                d  Disarm             r  Run profile
  t  Tare / zero thrust k  Clear latch        b  Abort profile
  +/-  throttle +/-25us    SPACE or x  E-STOP     q  quit
"""

    def __init__(self, ctl):
        self.ctl = ctl
        self.pwm = IDLE_US
        self.last = {"thrust_g": 0, "amps": 0, "amps_raw": 0,
                     "watts_est": 0, "pwm": IDLE_US}
        self.lines = []

    def _key(self):
        if not sys.stdin.isatty():
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None

    def _handle(self, ch):
        c = self.ctl
        if ch in (" ", "x"):
            c.post("estop")
        elif ch == "t":
            c.post("tare")
        elif ch == "k":
            c.post("clear")
        elif ch == "a":
            c.post("arm")
        elif ch == "r":
            c.post("run")
        elif ch == "d":
            c.post("disarm")
        elif ch == "b":
            c.post("abort")
        elif ch in "+=":
            self.pwm = min(MAX_US, self.pwm + 25)
            c.post("pwm", us=self.pwm)
        elif ch in "-_":
            self.pwm = max(IDLE_US, self.pwm - 25)
            c.post("pwm", us=self.pwm)
        elif ch == "q":
            return False
        return True

    def run(self):
        import termios
        import tty
        fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        old = termios.tcgetattr(fd) if fd is not None else None
        if fd is not None:
            tty.setcbreak(fd)
        try:
            while True:
                self.ctl.beat()
                ch = self._key()
                if ch is not None and not self._handle(ch):
                    break
                while True:
                    try:
                        self.last = self.ctl.samples.get_nowait()
                    except queue.Empty:
                        break
                while True:
                    try:
                        self.lines.append(self.ctl.messages.get_nowait())
                    except queue.Empty:
                        break
                self.lines = self.lines[-8:]
                self._draw()
                time.sleep(0.1)
        finally:
            if fd is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _draw(self):
        st = self.ctl.snapshot()
        banner = ("CONNECTING TO HARDWARE..." if st["phase"] == PHASE_DISCONNECTED
                  else "!!! MOTOR CAN SPIN !!!")
        flag = (f"LATCHED[{st['reason']}]" if st["latched"]
                else st["phase"].upper())
        s = self.last
        os.system("clear")
        print("=" * 78)
        print(banner.center(78))
        print("=" * 78)
        print(f" {flag:<34} cell:{st['cell'] or '-':<16} "
              f"power:{st['power'].upper()}")
        g = s.get("thrust_g", 0) or 0.0
        print(f" thrust {g:8.1f} g  = {g / G_PER_KG:6.3f} kg "
              f"= {g / G_PER_LB:6.3f} lb")
        print(f" amps   {s.get('amps', 0):8.2f} A     "
              f"pwm {s.get('pwm', IDLE_US)}")
        print(f" rows {st['rows']}   bad {st['bad']}   "
              f"step {st['step'] + 1 if st['steps'] else 0}/{st['steps']}")
        print("-" * 78)
        print(self.KEYS)
        print("-" * 78)
        for ln in self.lines:
            print(f"  {ln}")


# =========================================================================
# Desktop interface: HDMI monitor, USB keyboard and mouse
# =========================================================================
class DesktopUI:
    """Console for a monitor, keyboard and mouse wired to the Pi."""

    def __init__(self, ctl, screen=(1280, 720), fullscreen=False,
                 fps=8, points=200, plots=2, use_blit=True):
        import matplotlib
        matplotlib.use("TkAgg")
        matplotlib.rcParams["toolbar"] = "None"
        matplotlib.rcParams["path.simplify"] = True
        matplotlib.rcParams["path.simplify_threshold"] = 1.0
        matplotlib.rcParams["agg.path.chunksize"] = 10000
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        from matplotlib.widgets import Button, Slider, TextBox
        from matplotlib.patches import Circle
        self.plt, self.animation = plt, animation
        self.Button, self.Slider, self.TextBox = Button, Slider, TextBox
        self.Circle = Circle

        self.ctl = ctl
        self.status = "ready"
        self.pwm = IDLE_US
        self.typing = False
        self.interval = max(40, int(1000 / max(fps, 1)))
        self.n_plots = plots
        self.use_blit = use_blit
        self._bg = None
        self._last_capture = 0.0
        self._min_recapture_s = 0.15

        n = points
        self.SPAN = points
        self.series = {k: [0.0] * n for k in
                       ("thrust_g", "watts_est", "amps", "amps_raw")}
        xs = list(range(-n + 1, 1))

        w, h = screen
        self.fig = plt.figure(figsize=(w / 100.0, h / 100.0), dpi=100)
        self.fig.patch.set_facecolor("#111418")
        try:
            self.fig.canvas.manager.set_window_title("testCell")
        except Exception:
            pass
        if fullscreen:
            for attempt in (
                lambda: self.fig.canvas.manager.window.attributes(
                    "-fullscreen", True),
                lambda: self.fig.canvas.manager.full_screen_toggle(),
            ):
                try:
                    attempt()
                    break
                except Exception:
                    continue

        def mkax(rect, ylabel, ylim, color, key):
            ax = self.fig.add_axes(rect)
            ax.set_facecolor("#181c22")
            ax.set_ylim(ylim)
            ax.set_xlim(xs[0], xs[-1])
            ax.grid(alpha=0.25, color="0.5")
            ax.tick_params(colors="0.75", labelsize=7)
            for sp in ax.spines.values():
                sp.set_color("0.4")
            ax.set_ylabel(ylabel, color="0.85", fontsize=8)
            ln, = ax.plot(xs, self.series[key], color=color, lw=1.3)
            if key == "thrust_g":
                lb = ax.twinx()
                lb.set_ylim(ylim[0] / G_PER_LB, ylim[1] / G_PER_LB)
                lb.set_ylabel("lb", color="0.85", fontsize=8)
                lb.tick_params(colors="0.75", labelsize=7)
                for sp in lb.spines.values():
                    sp.set_color("0.4")
                lb.patch.set_visible(False)
            return ln

        if self.n_plots >= 4:
            self.lines = [
                mkax([0.05, 0.68, 0.235, 0.22], "Thrust (g)", (0, 3500),
                     "#4fc3f7", "thrust_g"),
                mkax([0.385, 0.68, 0.235, 0.22], "Est. power (W)", (0, 1200),
                     "#81c784", "watts_est"),
                mkax([0.05, 0.40, 0.235, 0.22], "Current OU2 filtered (A)",
                     (0, 50), "#e57373", "amps"),
                mkax([0.385, 0.40, 0.235, 0.22], "Current OU1 raw (A)",
                     (0, 50), "#ffb74d", "amps_raw"),
            ]
            self.keys = ["thrust_g", "watts_est", "amps", "amps_raw"]
        else:
            self.lines = [
                mkax([0.05, 0.55, 0.545, 0.35], "Thrust (g)", (0, 3500),
                     "#4fc3f7", "thrust_g"),
                mkax([0.05, 0.40, 0.545, 0.13], "Current OU2 (A)", (0, 50),
                     "#e57373", "amps"),
            ]
            self.keys = ["thrust_g", "amps"]

        self.banner = self.fig.text(0.5, 0.960, "", ha="center", fontsize=14,
                                    family="monospace", fontweight="bold")
        self.status_t = self.fig.text(0.5, 0.925, "", ha="center", fontsize=9,
                                      family="monospace", color="0.8")

        def readout(x, y, label):
            self.fig.text(x, y + 0.062, label, fontsize=9, color="0.65",
                          family="monospace")
            return self.fig.text(x, y, "--", fontsize=24, color="#eceff1",
                                 family="monospace", fontweight="bold")

        show4 = self.n_plots >= 4
        self.r_thrust = readout(0.67, 0.795, "THRUST  grams")
        self.r_thrust_alt = self.fig.text(0.67, 0.757, "", fontsize=11,
                                          color="#90a4ae",
                                          family="monospace")
        if show4:
            self.r_watts = readout(0.845, 0.795, "EST POWER  W")
            self.r_amps = readout(0.67, 0.645, "AMPS  OU2")
            self.r_volts = readout(0.845, 0.645, "AMPS  OU1")
        else:
            self.r_watts = None
            self.r_amps = readout(0.67, 0.645, "AMPS")
            self.r_volts = None
        self.r_info = self.fig.text(0.67, 0.565, "", fontsize=10,
                                    color="0.75", family="monospace")
        self.r_cell = self.fig.text(0.67, 0.520, "", fontsize=10,
                                    color="#ffd54f", family="monospace")

        # Three lamps: CONN / ARM / RUN. CONN is amber while still connecting
        # (not grey) so the automatic background retry is visible without
        # reading text. A latched stop turns every lamp red.
        lamp_y = 0.455
        self.phase_lamps = []
        for i, lbl in enumerate(("CONN", "ARM", "RUN")):
            x = 0.72 + i * 0.07
            lamp = self.Circle((x, lamp_y), 0.016,
                               transform=self.fig.transFigure,
                               facecolor="#37474f", edgecolor="0.5",
                               linewidth=1.0, zorder=5)
            self.fig.add_artist(lamp)
            self.fig.text(x, lamp_y - 0.030, lbl, ha="center", fontsize=7,
                          color="0.55", family="monospace")
            self.phase_lamps.append(lamp)

        self.power_lamp = self.Circle((0.94, lamp_y), 0.016,
                                      transform=self.fig.transFigure,
                                      facecolor="#ffb74d", edgecolor="0.5",
                                      linewidth=1.2, zorder=5)
        self.fig.add_artist(self.power_lamp)
        self.fig.text(0.94, lamp_y - 0.032, "POWER", ha="center",
                      fontsize=7, color="0.55", family="monospace")

        self._dyn_artists = [*self.lines, self.banner, self.status_t,
                             self.r_thrust, self.r_thrust_alt, self.r_amps,
                             self.r_info, self.r_cell, self.power_lamp,
                             *self.phase_lamps]
        if self.r_watts is not None:
            self._dyn_artists.append(self.r_watts)
        if self.r_volts is not None:
            self._dyn_artists.append(self.r_volts)
        self._redraw_guard = False

        self._text_fields()
        self._controls()
        self.fig.text(0.05, 0.005,
                      "keys   a ARM   r RUN   d DISARM   t TARE   "
                      "k CLEAR   b ABORT   +/- throttle   q quit   "
                      "SPACE/ESC E-STOP",
                      fontsize=8, color="0.55", family="monospace")

        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self.fig.canvas.mpl_connect("close_event", lambda e: ctl.post("estop"))

    def _text_fields(self):
        self.boxes = {}
        fields = [("label", "Run", self.ctl.args.label_value, 0.075),
                  ("name", "Cell", self.ctl.args.cell["name"], 0.29),
                  ("motor", "Motor", self.ctl.args.cell["motor"], 0.505),
                  ("prop", "Prop", self.ctl.args.cell["prop"], 0.72)]
        for key, label, init, x in fields:
            ax = self.fig.add_axes([x, 0.315, 0.145, 0.045])
            tb = self.TextBox(ax, label + "  ", initial=str(init),
                              color="#263238", hovercolor="#37474f")
            tb.label.set_color("0.75")
            tb.label.set_fontsize(9)
            tb.text_disp.set_color("#eceff1")
            tb.on_submit(lambda text, k=key: self._set_field(k, text))
            self.boxes[key] = tb

    def _set_field(self, key, text):
        text = (text or "").strip()
        if not text:
            return
        st = self.ctl.snapshot()
        if st["phase"] in (PHASE_ARMED, PHASE_RUNNING):
            self.status = "disarm before editing the cell"
            return
        if key == "label":
            self.ctl.args.label_value = text
            self.ctl.post("relabel", label=text)
        else:
            self.ctl.post("setcell", key=key, value=text)
            self.status = f"{key} = {text}"

    def _controls(self):
        ax = self.fig.add_axes([0.78, 0.05, 0.185, 0.20])
        self.b_estop = self.Button(ax, "E-STOP", color="#c62828",
                                   hovercolor="#e53935")
        self.b_estop.label.set_fontsize(26)
        self.b_estop.label.set_fontweight("bold")
        self.b_estop.label.set_color("white")
        self.b_estop.on_clicked(lambda e: self._estop())

        cols = [0.05 + i * 0.118 for i in range(6)]
        self.buttons = {}

        def mk(key, col, row, text, cb, color="#37474f"):
            a = self.fig.add_axes([cols[col], 0.16 if row == 0 else 0.055,
                                   0.105, 0.085])
            b = self.Button(a, text, color=color, hovercolor="#546e7a")
            b.label.set_fontsize(9)
            b.label.set_color("white")
            b.label.set_fontweight("bold")
            b.on_clicked(cb)
            self.buttons[key] = (b, text, color)

        # One row now, not two: with SIM, LIVE, INIT ENV and INIT CELL all
        # gone there is no sequence left to teach, so the remaining controls
        # sit on a single line in the order they are actually used.
        mk("arm", 0, 1, "ARM  (a)", lambda e: self.ctl.post("arm"),
           "#2e7d32")
        mk("run", 1, 1, "RUN  (r)", lambda e: self.ctl.post("run"),
           "#2e7d32")
        mk("disarm", 2, 1, "DISARM  (d)", lambda e: self.ctl.post("disarm"))
        mk("abort", 3, 1, "ABORT  (b)", lambda e: self.ctl.post("abort"))
        mk("tare", 4, 1, "TARE  (t)", lambda e: self.ctl.post("tare"),
           "#455a64")
        mk("clear", 5, 1, "CLEAR  (k)", lambda e: self.ctl.post("clear"))

        # Throttle nudge buttons move up to the freed row, beside the slider
        # they act on.
        mk("down", 4, 0, "THR  -", lambda e: self._throttle(-25), "#455a64")
        mk("up", 5, 0, "THR  +", lambda e: self._throttle(+25), "#455a64")

        ax_sl = self.fig.add_axes([0.09, 0.265, 0.53, 0.025])
        self.sl = self.Slider(ax_sl, "PWM us", IDLE_US, MAX_US,
                              valinit=IDLE_US, valstep=5, color="#455a64")
        self.sl.label.set_color("0.75")
        self.sl.label.set_fontsize(9)
        self.sl.valtext.set_color("#eceff1")
        self.sl.on_changed(self._slider)

    def _slider(self, val):
        self.pwm = int(val)
        self.ctl.post("pwm", us=self.pwm)

    def _estop(self):
        self.ctl.post("estop")
        self.pwm = IDLE_US
        try:
            self.sl.set_val(IDLE_US)
        except Exception:
            pass

    def _throttle(self, delta):
        self.pwm = max(IDLE_US, min(MAX_US, self.pwm + delta))
        try:
            self.sl.set_val(self.pwm)
        except Exception:
            self.ctl.post("pwm", us=self.pwm)

    def _key(self, event):
        k = event.key
        if k in (" ", "escape"):
            self._estop()
            return
        mapping = {"t": ("tare", {}), "k": ("clear", {}), "r": ("run", {}),
                   "d": ("disarm", {}), "b": ("abort", {}), "q": ("quit", {})}
        if k == "a":
            self.ctl.post("arm")
        elif k in ("+", "="):
            self._throttle(+25)
        elif k in ("-", "_"):
            self._throttle(-25)
        elif k in mapping:
            cmd, kw = mapping[k]
            self.ctl.post(cmd, **kw)
            if cmd == "quit":
                self.plt.close(self.fig)

    def _frame(self, _i):
        self.ctl.beat()
        last = None
        while True:
            try:
                last = self.ctl.samples.get_nowait()
            except queue.Empty:
                break
            for k in self.series:
                buf = self.series[k]
                buf.pop(0)
                buf.append(float(last.get(k) or 0.0))
        while True:
            try:
                self.status = self.ctl.messages.get_nowait()
            except queue.Empty:
                break

        for ln, k in zip(self.lines, self.keys):
            ln.set_ydata(self.series[k])

        st = self.ctl.snapshot()
        if st["phase"] == PHASE_DISCONNECTED:
            self.banner.set_text("CONNECTING TO HARDWARE...")
            self.banner.set_color("#ffa726")
        else:
            self.banner.set_text("L I V E   -   M O T O R   C A N   S P I N")
            self.banner.set_color("#ef5350")

        step = {PHASE_READY: 0, PHASE_ARMED: 1, PHASE_RUNNING: 2}.get(
            st["phase"], -1)
        for i, lamp in enumerate(self.phase_lamps):
            if st["latched"]:
                lamp.set_facecolor("#ef5350")
            elif st["phase"] == PHASE_DISCONNECTED:
                lamp.set_facecolor("#ffa726" if i == 0 else "#37474f")
            else:
                lamp.set_facecolor("#66bb6a" if i <= step else "#37474f")

        reason = f"stopped: {st['reason']}   " if st["latched"] else ""
        self.status_t.set_text(f"{reason}bad:{st['bad']}   |   {self.status}")

        state = st["power"]
        self.power_lamp.set_facecolor(
            {"present": "#66bb6a", "removed": "#ef5350"}.get(state,
                                                              "#ffb74d"))

        g = self.series["thrust_g"][-1]
        self.r_thrust.set_text(f"{g:.0f}")
        self.r_thrust_alt.set_text(f"{g / G_PER_KG:6.3f} kg   "
                                   f"{g / G_PER_LB:6.3f} lb")
        if self.r_watts is not None:
            self.r_watts.set_text(f"{self.series['watts_est'][-1]:.0f}")
        self.r_amps.set_text(f"{self.series['amps'][-1]:.1f}")
        if self.r_volts is not None:
            self.r_volts.set_text(f"{self.series['amps_raw'][-1]:.1f}")
        self.r_info.set_text(f"pwm {self.pwm}   ripple {st['ripple']:.2f} A   "
                             f"rows {st['rows']}   "
                             f"step {st['step'] + 1 if st['steps'] else 0}/"
                             f"{st['steps']}")
        cur = self.ctl.args.cell
        self.r_cell.set_text(f"{cur['name']}  {cur['motor']}  {cur['prop']}")

        for key, (btn, text, color) in self.buttons.items():
            if btn.label.get_text() != text:
                btn.label.set_text(text)
                btn.color = color
                btn.ax.set_facecolor(color)
                self.fig.canvas.draw_idle()
                self._last_capture = 0.0

        artists = [*self.lines, self.banner, self.status_t, self.r_thrust,
                  self.r_thrust_alt, self.r_amps, self.r_info, self.r_cell,
                  self.power_lamp, *self.phase_lamps]
        if self.r_watts is not None:
            artists.append(self.r_watts)
        if self.r_volts is not None:
            artists.append(self.r_volts)
        return artists

    def run(self):
        if not self.use_blit:
            self._ani = self.animation.FuncAnimation(
                self.fig, self._frame, interval=self.interval, blit=False,
                cache_frame_data=False)
            self.plt.show(block=True)
            return

        self._bg = None
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        timer = self.fig.canvas.new_timer(interval=self.interval)
        timer.add_callback(self._tick)
        timer.start()
        self._timer = timer
        self.plt.show(block=True)

    def _on_draw(self, _event):
        if self._redraw_guard:
            return
        now = time.monotonic()
        if (self._bg is not None
                and now - self._last_capture < self._min_recapture_s):
            return
        self._redraw_guard = True
        try:
            visible = [a.get_visible() for a in self._dyn_artists]
            for a in self._dyn_artists:
                a.set_visible(False)
            self.fig.canvas.draw()
            self._bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            self._last_capture = now
            for a, v in zip(self._dyn_artists, visible):
                a.set_visible(v)
        finally:
            self._redraw_guard = False

    def _tick(self):
        c = self.fig.canvas
        if self._bg is None:
            c.draw()
            return True
        try:
            artists = self._frame(0)
            c.restore_region(self._bg)
            for a in artists:
                (a.axes or self.fig).draw_artist(a)
            c.blit(self.fig.bbox)
        except Exception as exc:
            log_exception("UI tick failed", exc)
            self._bg = None
        return True


# =========================================================================
def have_display():
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        try:
            import matplotlib  # noqa: F401
            import tkinter     # noqa: F401
            return True
        except ImportError:
            return False
    return False


def main():
    ap = argparse.ArgumentParser(
        description="testCell -- self-contained thrust stand controller")
    ap.add_argument("--headless", action="store_true",
                    help="force the terminal interface (for SSH)")
    ap.add_argument("--fullscreen", action="store_true",
                    help="fill the HDMI monitor; windowed by default")
    ap.add_argument("--screen", default="1280x720",
                    help="window size, e.g. 1280x720 or 1920x1080")
    ap.add_argument("--label", default=None,
                    help="run label; generated automatically if unset")
    ap.add_argument("--cell-name", default="cell-1")
    ap.add_argument("--motor", default="unspecified")
    ap.add_argument("--prop", default="unspecified")
    ap.add_argument("--supply", default="6S 25.2V")
    ap.add_argument("--backend", choices=["direct", "serial"],
                    default="direct",
                    help="direct: Pi wired to HX711 and ADS1115. "
                         "serial: telemetry from a microcontroller on USB.")
    ap.add_argument("--serial-port", default=SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--hx-dt", type=int, default=HX_DT_PIN)
    ap.add_argument("--hx-sck", type=int, default=HX_SCK_PIN)
    ap.add_argument("--i2c-bus", type=int, default=I2C_BUS)
    ap.add_argument("--ads-addr", type=lambda v: int(v, 0), default=ADS_ADDR)
    ap.add_argument("--pwm-gpio", type=int, default=PWM_GPIO)
    ap.add_argument("--fps", type=int, default=8,
                    help="console refresh rate; lower it if the window feels "
                         "sluggish")
    ap.add_argument("--plot-points", type=int, default=200,
                    help="samples kept on each plot; fewer redraws faster")
    ap.add_argument("--plots", type=int, choices=[2, 4], default=2,
                    help="2 (default): thrust and current only. 4: also "
                         "shows estimated power and the raw/filtered "
                         "current comparison. Logged data is the same "
                         "either way.")
    ap.add_argument("--no-blit", action="store_true",
                    help="full redraws instead of blitting; much slower, use "
                         "only if the display shows artefacts")
    ap.add_argument("--nominal-volts", type=float, default=NOMINAL_VOLTS,
                    help="pack voltage used to estimate power; there is no "
                         "voltage sensor, so this is an assumption not a "
                         "measurement")
    ap.add_argument("--no-current-amps", type=float, default=NO_CURRENT_A,
                    help="below this current, throttle commanded with no "
                         "sense wire reads as motor power removed; must sit "
                         "below the real current draw at your lowest test "
                         "throttle or low-power runs will false-trip")
    args = ap.parse_args()

    log_path = setup_logging()
    LOG.info("args: %s", vars(args))
    if log_path:
        print(f"testCell {SCRIPT_VERSION}  (log: {log_path})")
    else:
        print(f"testCell {SCRIPT_VERSION}  (log file unavailable, see above)")
    print("connecting to hardware...")

    args.profile = DEFAULT_PROFILE

    args.cell_presets = list(CELL_PRESETS)
    preset_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "testCell_cells.json")
    if os.path.exists(preset_file):
        try:
            import json
            with open(preset_file) as fh:
                loaded = json.load(fh)
            if isinstance(loaded, list) and loaded:
                args.cell_presets = loaded
        except Exception as exc:
            log_exception(f"ignoring {preset_file}", exc)
            print(f"ignoring {preset_file}: {exc}")

    if args.cell_name != "cell-1" or args.motor != "unspecified":
        args.cell = {"name": args.cell_name, "motor": args.motor,
                     "prop": args.prop, "supply": args.supply}
    else:
        args.cell = dict(args.cell_presets[0])

    label = args.label or datetime.now().strftime("run-%Y%m%d")
    args.label_value = label
    recorder = Recorder(label)

    ctl = Controller(args, recorder)
    ctl.start()

    def on_signal(_s, _f):
        ctl.running = False
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        sw, sh = (int(v) for v in args.screen.lower().split("x"))
    except Exception:
        sw, sh = 1280, 720

    ui = None
    if not args.headless and have_display():
        try:
            ui = DesktopUI(ctl, screen=(sw, sh), fullscreen=args.fullscreen,
                           fps=args.fps, points=args.plot_points,
                           plots=args.plots, use_blit=not args.no_blit)
        except Exception as exc:
            log_exception("desktop console unavailable, falling back to "
                         "terminal", exc)
            print(f"desktop console unavailable ({exc}); using terminal")
    if ui is None:
        ui = TerminalUI(ctl)

    try:
        ui.run()
    except KeyboardInterrupt:
        LOG.info("stopped by KeyboardInterrupt")
    finally:
        ctl.shutdown()
        if recorder.path:
            print(f"\nwrote {recorder.rows} rows to {recorder.path}")
        LOG.info("session ended  (rows written: %d)", recorder.rows)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as exc:
        setup_logging()
        log_exception("unhandled exception, process exiting", exc)
        raise
