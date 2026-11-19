#!/usr/bin/env python3
"""
motorRamp.py -- simple motor integrity ramp (NO PROP).

Ramps throttle 0 -> 50% in 5% steps, then 50 -> 0% in 5% steps.
ESC wiring is the same as testCell v6.3:
    ESC signal -> GPIO 12 (header pin 32), hardware PWM
    ESC ground -> GND     (header pin 34)
    Requires 'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' in config.txt

If no hardware PWM is found (e.g. running on a laptop) it runs in SIM mode
and just shows the sequence.

Run:  python3 motorRamp.py
"""
import glob
import os
import tkinter as tk
from tkinter import ttk

# ---------------- settings ----------------
PWM_GPIO = 12          # BCM 12, header pin 32
IDLE_US = 1000         # 0% throttle
MAX_US = 2000          # 100% throttle
STEP_PCT = 5           # increment
TOP_PCT = 50           # peak
HOLD_S = 4             # seconds at each step
ARM_S = 3              # seconds at 0% before ramp so the ESC arms

# 5,10,...,50, 45,...,5, 0
UP = list(range(STEP_PCT, TOP_PCT + 1, STEP_PCT))
DOWN = list(range(TOP_PCT - STEP_PCT, -1, -STEP_PCT))
SEQUENCE = UP + DOWN


def pct_to_us(pct):
    return int(IDLE_US + (MAX_US - IDLE_US) * pct / 100)


# ---------------- hardware PWM (same method as testCell v6.3) ----------------
class ESC:
    def __init__(self):
        self.path = None
        channel = 0 if PWM_GPIO in (12, 18) else 1
        for chip in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
            try:
                with open(os.path.join(chip, "npwm")) as f:
                    if int(f.read()) <= channel:
                        continue
                p = os.path.join(chip, f"pwm{channel}")
                if not os.path.exists(p):
                    with open(os.path.join(chip, "export"), "w") as f:
                        f.write(str(channel))
                    import time; time.sleep(0.1)
                if os.path.exists(p):
                    self.path = p
                    self._w("period", 20_000_000)          # 50 Hz
                    self._w("duty_cycle", IDLE_US * 1000)
                    self._w("enable", 1)
                    break
            except OSError:
                continue
        self.sim = self.path is None

    def _w(self, attr, val):
        with open(os.path.join(self.path, attr), "w") as f:
            f.write(str(int(val)))

    def set_pct(self, pct):
        if not self.sim:
            self._w("duty_cycle", pct_to_us(pct) * 1000)

    def close(self):
        if not self.sim:
            self.set_pct(0)
            self._w("enable", 0)


# ---------------- GUI ----------------
class App:
    def __init__(self, root):
        self.root = root
        self.esc = ESC()
        self.job = None
        self.idx = -1
        self.left = 0

        root.title("Motor Ramp Test")
        root.configure(padx=20, pady=20)

        mode = "SIM - NO PWM FOUND" if self.esc.sim else "LIVE - GPIO 12"
        tk.Label(root, text=mode, font=("Arial", 14, "bold"),
                 fg="orange" if self.esc.sim else "red").pack()

        self.state = tk.Label(root, text="IDLE", font=("Arial", 22, "bold"))
        self.state.pack(pady=(10, 0))
        self.big = tk.Label(root, text="0 %", font=("Arial", 64, "bold"))
        self.big.pack()
        self.info = tk.Label(root, text=f"{IDLE_US} us", font=("Arial", 14))
        self.info.pack()

        # one box per step, current one highlighted
        row = tk.Frame(root)
        row.pack(pady=10)
        self.boxes = []
        for p in SEQUENCE:
            b = tk.Label(row, text=str(p), width=3, relief="solid",
                         bg="white", font=("Arial", 11))
            b.pack(side="left", padx=1)
            self.boxes.append(b)

        self.bar = ttk.Progressbar(root, length=600, maximum=len(SEQUENCE))
        self.bar.pack(pady=5)

        btns = tk.Frame(root)
        btns.pack(pady=10)
        self.start_btn = tk.Button(btns, text="START", width=12, height=2,
                                   bg="#2e7d32", fg="white",
                                   font=("Arial", 16, "bold"), command=self.start)
        self.start_btn.pack(side="left", padx=10)
        tk.Button(btns, text="STOP", width=12, height=2, bg="#c62828",
                  fg="white", font=("Arial", 16, "bold"),
                  command=self.stop).pack(side="left", padx=10)

        root.bind("<space>", lambda e: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.quit)

    def show(self, pct, label):
        self.big.config(text=f"{pct} %")
        self.state.config(text=label)
        self.info.config(text=f"{pct_to_us(pct)} us   |   {self.left} s left in step")
        for i, b in enumerate(self.boxes):
            if i < self.idx:
                b.config(bg="#a5d6a7")       # done
            elif i == self.idx:
                b.config(bg="#ffeb3b")       # current
            else:
                b.config(bg="white")
        self.bar["value"] = max(self.idx, 0)

    def start(self):
        if self.job:
            return
        self.start_btn.config(state="disabled")
        self.idx = -1                        # -1 = arming at 0%
        self.left = ARM_S
        self.esc.set_pct(0)
        self.tick()

    def tick(self):
        if self.left <= 0:
            self.idx += 1
            if self.idx >= len(SEQUENCE):
                self.finish("COMPLETE")
                return
            self.left = HOLD_S
            self.esc.set_pct(SEQUENCE[self.idx])

        if self.idx < 0:
            self.show(0, "ARMING ESC")
        else:
            phase = "RAMP UP" if self.idx < len(UP) else "RAMP DOWN"
            self.show(SEQUENCE[self.idx],
                      f"{phase}  -  step {self.idx + 1} of {len(SEQUENCE)}")
        self.left -= 1
        self.job = self.root.after(1000, self.tick)

    def stop(self):
        self.finish("STOPPED")

    def finish(self, label):
        if self.job:
            self.root.after_cancel(self.job)
            self.job = None
        self.esc.set_pct(0)
        self.left = 0
        if label == "COMPLETE":
            self.idx = len(SEQUENCE)
            self.bar["value"] = len(SEQUENCE)
        self.show(0, label)
        self.start_btn.config(state="normal")

    def quit(self):
        self.finish("STOPPED")
        self.esc.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
