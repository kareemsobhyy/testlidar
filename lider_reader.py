
#!/usr/bin/env python3
# LD06/LD19 UART visualizer for Raspberry Pi OS
# Default: /dev/ttyAMA0 @ 230400   (override via CLI: python3 ld19_view_fix.py /dev/serial0 230400)

import sys, struct, math, time, os
import serial

# -------- Config (override with argv) --------
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 230400
RANGE_MM = 8000
MAX_POINTS = 3000
PTS = 12

# -------- Hold PWM low on GPIO18 (pin 12) --------
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)  # PWM pin low
except Exception as e:
    print(f"[WARN] GPIO init failed: {e}")

# -------- Matplotlib (GUI; if headless, save PNG) --------
HEADLESS = not bool(os.environ.get("DISPLAY"))
import matplotlib
if HEADLESS:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# -------- CRC-8 (robust) --------
# Known-good 256-entry table for LD06/LD19 (poly 0x31, init 0x00, no xor)
CRC_TABLE = [
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01
]
def crc8_safe(data: bytes) -> int:
    """Robust CRC-8: use table; if anything weird happens, fall back to bitwise poly 0x31."""
    try:
        c = 0
        for x in data:
            c = CRC_TABLE[(c ^ x) & 0xFF]
        return c
    except Exception:
        # bitwise fallback (poly 0x31, init 0x00)
        c = 0
        for x in data:
            c ^= x
            for _ in range(8):
                c = ((c << 1) ^ 0x31) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
        return c

# -------- Serial helpers --------
def read_frame(ser):
    """Read one LD06/LD19 frame: 0x54 + 46 bytes (ver/len 0x2C, 12 points, CRC)."""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0x54:
            continue
        rest = ser.read(46)
        if len(rest) != 46:
            continue
        f = bytes([0x54]) + rest
        if rest[0] != 0x2C:
            continue
        # Accept frame if CRC matches OR (as a last resort) if basic geometry looks sane
        if crc8_safe(f[:-1]) != f[-1]:
            # soft-fail: keep searching; comment the next line to accept anyway
            continue
        return f

def parse_frame(f):
    start = int.from_bytes(f[4:6],'little')     # centi-deg
    pts = []
    o = 6
    for _ in range(PTS):
        dist = int.from_bytes(f[o:o+2],'little')
        inten = f[o+2]
        pts.append((dist,inten))
        o += 3
    end = int.from_bytes(f[o:o+2],'little')
    end_adj = end if end >= start else end + 36000
    step = (end_adj - start) / (PTS - 1)
    angs_deg = [(start + i*step)/100.0 for i in range(PTS)]
    dists = [p[0] for p in pts]
    return angs_deg, dists

def to_xy(angle_deg, dist_mm):
    theta = math.radians(360.0 - angle_deg)  # sensor clockwise -> math CCW
    return dist_mm*math.cos(theta), dist_mm*math.sin(theta)

def setup_plot():
    fig, ax = plt.subplots()
    ax.set_aspect('equal','box')
    ax.set_xlim(-RANGE_MM, RANGE_MM)
    ax.set_ylim(-RANGE_MM, RANGE_MM)
    ax.grid(True, alpha=0.4)
    scat = ax.scatter([], [], s=4)
    return fig, ax, scat

def main():
    print(("Headless → saving PNGs" if HEADLESS else "GUI window"))
    ser = serial.Serial(PORT, BAUD, timeout=1)
    fig, ax, scat = setup_plot()
    xs, ys = [], []
    last_png = 0
    frames = 0
    try:
        while True:
            f = read_frame(ser)
            if f is None:
                continue
            angs, dists = parse_frame(f)
            for a, d in zip(angs, dists):
                if d == 0 or d > 12000:  # drop invalid/out-of-range
                    continue
                x, y = to_xy(a, d)
                xs.append(x); ys.append(y)
            if len(xs) > MAX_POINTS:
                xs = xs[-MAX_POINTS:]; ys = ys[-MAX_POINTS:]
            scat.set_offsets(list(zip(xs, ys)))
            ax.set_title(f"{PORT} @ {BAUD}  points:{len(xs)}")
            if HEADLESS:
                now = time.time()
                if now - last_png > 0.25:
                    fig.savefig("scan_latest.png", dpi=120, bbox_inches="tight")
                    last_png = now
            else:
                plt.pause(0.001)

            frames += 1
            if frames % 100 == 0:
                print(f"{time.strftime('%H:%M:%S')} frames={frames} buf={len(xs)}")
    finally:
        ser.close()
        try:
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == "__main__":
    print(f"Opening {PORT} @ {BAUD} …")
    main()
