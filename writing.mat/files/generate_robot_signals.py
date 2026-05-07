"""
Delta Robot Signal Builder .mat File Generator
Grid: 26x26 cm, origin at center => X,Y in [-0.13, +0.13] m
Z_draw = 0.25 m (pen DOWN, +Z is down)
Z_lift = 0.13 m (pen UP)
dt = 0.01 s (100 Hz)
"""

import numpy as np
from scipy.io import savemat
import os

# ── Settings ────────────────────────────────────────────────
Z_DRAW   = 0.25      # pen touching paper (+Z is down)
Z_LIFT   = 0.13      # pen clear of paper
DT       = 0.01      # 100 Hz
T_MOVE   = 0.40      # seconds to travel between strokes (lifted)
T_SEG    = 0.50      # seconds to draw one segment
T_PENLIFT= 0.20      # seconds to raise/lower pen
GRID     = 0.26      # 26 cm

NAMES = ['AHMED', 'NADA', 'HAMZA', 'BOODY', 'POWER', 'SH7S']

# ── Letter stroke definitions ────────────────────────────────
# Each letter returns a list of strokes.
# Each stroke is a numpy array of shape (N,2) — the pen draws
# continuously through each stroke, then lifts between strokes.

def arc(cx, cy, rx, ry, a_start, a_end, n=24):
    t = np.linspace(a_start, a_end, n)
    return np.column_stack([cx + rx*np.cos(t), cy + ry*np.sin(t)])

def letter_strokes(ch, x0, x1, y0, y1):
    xm = (x0+x1)/2
    ym = (y0+y1)/2
    w  = x1-x0
    h  = y1-y0
    ch = ch.upper()

    if ch == 'A':
        cross_y = y0 + h*0.45
        return [
            np.array([[x0,y0],[xm,y1],[x1,y0]]),
            np.array([[x0+w*0.22, cross_y],[x1-w*0.22, cross_y]])
        ]
    elif ch == 'H':
        return [
            np.array([[x0,y0],[x0,y1]]),
            np.array([[x0,ym],[x1,ym]]),
            np.array([[x1,y0],[x1,y1]])
        ]
    elif ch == 'M':
        return [
            np.array([[x0,y0],[x0,y1],[xm,y0+h*0.4],[x1,y1],[x1,y0]])
        ]
    elif ch == 'E':
        return [
            np.array([[x1,y1],[x0,y1],[x0,y0],[x1,y0]]),
            np.array([[x0,ym],[x0+w*0.75,ym]])
        ]
    elif ch == 'D':
        pts = arc(x0+w*0.15, ym, w*0.45, h*0.5, -np.pi/2, np.pi/2, 24)
        pts = np.vstack([[[x0,y1],[x0,y0]], pts[::-1]])
        return [pts]
    elif ch == 'N':
        return [
            np.array([[x0,y0],[x0,y1],[x1,y0],[x1,y1]])
        ]
    elif ch == 'B':
        top_bump = arc(x0+w*0.35, y0+h*0.75, w*0.35, h*0.25, -np.pi/2, np.pi/2, 16)
        bot_bump = arc(x0+w*0.35, y0+h*0.25, w*0.40, h*0.25, -np.pi/2, np.pi/2, 16)
        pts = np.vstack([
            [[x0,y0],[x0,y1]],
            top_bump,
            [[x0,ym]],
            bot_bump,
            [[x0,y0]]
        ])
        return [pts]
    elif ch == 'O':
        return [arc(xm, ym, w*0.5, h*0.5, 0, 2*np.pi, 36)]
    elif ch == 'Y':
        return [
            np.array([[x0,y1],[xm,ym],[x1,y1]]),
            np.array([[xm,ym],[xm,y0]])
        ]
    elif ch == 'P':
        bump = arc(x0+w*0.25, y0+h*0.72, w*0.38, h*0.26, -np.pi/2, np.pi/2, 16)
        pts = np.vstack([[[x0,y0],[x0,y1]], bump, [[x0,ym]]])
        return [pts]
    elif ch == 'W':
        return [
            np.array([[x0,y1],[x0+w*0.25,y0],[xm,y0+h*0.4],[x0+w*0.75,y0],[x1,y1]])
        ]
    elif ch == 'R':
        bump = arc(x0+w*0.25, y0+h*0.72, w*0.38, h*0.26, -np.pi/2, np.pi/2, 16)
        pts = np.vstack([[[x0,y0],[x0,y1]], bump, [[x0,ym]]])
        return [
            pts,
            np.array([[x0+w*0.1, ym],[x1,y0]])
        ]
    elif ch == 'S':
        top = arc(xm, y0+h*0.72, w*0.42, h*0.26,  np.pi/2, -np.pi/2+0.1, 18)
        bot = arc(xm, y0+h*0.28, w*0.42, h*0.26, -np.pi/2, np.pi/2-0.1,  18)
        return [np.vstack([top, bot])]
    elif ch == 'Z':
        return [
            np.array([[x0,y1],[x1,y1],[x0,y0],[x1,y0]])
        ]
    elif ch == '7':
        return [
            np.array([[x0,y1],[x1,y1],[x0+w*0.1,y0]])
        ]
    else:
        # fallback: draw bounding box
        return [np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]])]


# ── Signal builder helper ────────────────────────────────────

def interp_segment(p0, p1, duration):
    n = max(2, round(duration / DT))
    xs = np.linspace(p0[0], p1[0], n)
    ys = np.linspace(p0[1], p1[1], n)
    return xs, ys

def build_signal(word):
    n_letters = len(word)
    letter_w  = GRID / n_letters
    margin_x  = letter_w * 0.12
    margin_y  = GRID * 0.12

    # Collect all strokes with metadata
    all_strokes = []
    for li, ch in enumerate(word):
        x0 = -GRID/2 + li*letter_w + margin_x
        x1 = x0 + letter_w - 2*margin_x
        y0 = -GRID/2 + margin_y
        y1 =  GRID/2 - margin_y
        strokes = letter_strokes(ch, x0, x1, y0, y1)
        for s in strokes:
            all_strokes.append(s)

    X_list, Y_list, Z_list = [], [], []
    cur_x, cur_y, cur_z = 0.0, GRID/2, Z_LIFT  # start parked top-center lifted

    def add(xs, ys, zs):
        X_list.extend(xs); Y_list.extend(ys); Z_list.extend(zs)

    def ramp(n, z0, z1):
        return np.linspace(z0, z1, n)

    def flat(n, z):
        return np.full(n, z)

    for si, stroke in enumerate(all_strokes):
        target_x, target_y = stroke[0,0], stroke[0,1]

        # Make sure pen is lifted
        if cur_z < Z_LIFT - 1e-9:
            n = max(2, round(T_PENLIFT/DT))
            add([cur_x]*n, [cur_y]*n, ramp(n, cur_z, Z_LIFT))
            cur_z = Z_LIFT

        # Travel to stroke start (pen lifted)
        dist = np.hypot(target_x-cur_x, target_y-cur_y)
        travel_t = max(T_MOVE, dist / 0.10)   # min speed 10 cm/s
        n = max(2, round(travel_t/DT))
        xs = np.linspace(cur_x, target_x, n)
        ys = np.linspace(cur_y, target_y, n)
        add(xs, ys, flat(n, Z_LIFT))
        cur_x, cur_y = target_x, target_y

        # Lower pen
        n = max(2, round(T_PENLIFT/DT))
        add([cur_x]*n, [cur_y]*n, ramp(n, Z_LIFT, Z_DRAW))
        cur_z = Z_DRAW

        # Draw each segment of the stroke
        pts = stroke
        for pi in range(len(pts)-1):
            p0, p1 = pts[pi], pts[pi+1]
            seg_len = np.hypot(p1[0]-p0[0], p1[1]-p0[1])
            seg_t   = max(T_SEG, seg_len / 0.05)   # min speed 5 cm/s
            n = max(2, round(seg_t/DT))
            xs = np.linspace(p0[0], p1[0], n)
            ys = np.linspace(p0[1], p1[1], n)
            add(xs, ys, flat(n, Z_DRAW))
            cur_x, cur_y = p1[0], p1[1]

    # Finish: lift and park at origin
    n = max(2, round(T_PENLIFT/DT))
    add([cur_x]*n, [cur_y]*n, ramp(n, Z_DRAW, Z_LIFT))
    n = max(2, round(T_MOVE*2/DT))
    xs = np.linspace(cur_x, 0.0, n)
    ys = np.linspace(cur_y, 0.0, n)
    add(xs, ys, flat(n, Z_LIFT))

    X = np.array(X_list)
    Y = np.array(Y_list)
    Z = np.array(Z_list)
    T = np.arange(len(X)) * DT

    return T, X, Y, Z


# ── Generate & save all .mat files ──────────────────────────

out_dir = '/home/claude/robot_signals'
os.makedirs(out_dir, exist_ok=True)

for word in NAMES:
    print(f"Generating {word}...", end=' ', flush=True)
    T, X, Y, Z = build_signal(word)

    # Save in format compatible with MATLAB Signal Builder
    # Each signal is a column vector; time is also a column vector
    mat_data = {
        'time': T.reshape(-1,1),
        'X':    X.reshape(-1,1),
        'Y':    Y.reshape(-1,1),
        'Z':    Z.reshape(-1,1),
    }
    fpath = os.path.join(out_dir, f'{word}_signal.mat')
    savemat(fpath, mat_data)
    print(f"done — {len(T)} samples, {T[-1]:.1f}s")

print("\nAll 6 .mat files saved to:", out_dir)
