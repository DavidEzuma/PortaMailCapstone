#!/usr/bin/env python3
"""
PortaMail boot splash screen.
Displays a maroon 'Welcome to PortaMail' screen on /dev/fb0 with an animated
loading indicator. Runs until the X server socket appears or 90 s elapses.
"""

import os
import sys
import time
import struct

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    # Blank the framebuffer and exit if PIL unavailable
    try:
        with open('/dev/fb0', 'wb') as fb:
            fb.write(b'\x00' * (800 * 480 * 4))
    except Exception:
        pass
    sys.exit(0)

# ---------------------------------------------------------------------------
# Display config
# ---------------------------------------------------------------------------
FB_DEV   = '/dev/fb0'
W, H     = 800, 480

# Colours  (R, G, B)
MAROON_DARK  = (58,  0,   0)
MAROON_MID   = (80,  0,   0)
MAROON_LIGHT = (38,  0,   0)
WHITE        = (255, 255, 255)
GOLD         = (214, 211, 196)
GOLD_DIM     = (140, 138, 128)

FONT_PATH      = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def load_font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()


def make_gradient(w, h):
    """Simple top-left to bottom-right maroon gradient."""
    img = Image.new('RGB', (w, h))
    for y in range(h):
        t = y / h
        r = int(MAROON_DARK[0] + (MAROON_MID[0] - MAROON_DARK[0]) * t)
        g = 0
        b = 0
        for x in range(w):
            s = x / w
            rx = int(r + (MAROON_LIGHT[0] - r) * s * 0.4)
            img.putpixel((x, y), (max(0, min(255, rx)), g, b))
    return img


def image_to_fb_bytes(img):
    """Convert RGB PIL image to BGRA bytes for Linux framebuffer."""
    r, g, b = img.split()
    # Linux 32-bpp framebuffer is usually stored as BGRA in memory
    alpha = Image.new('L', img.size, 255)
    bgra = Image.merge('RGBA', (b, g, r, alpha))
    return bgra.tobytes()


def write_fb(data):
    try:
        with open(FB_DEV, 'wb') as fb:
            fb.write(data)
    except PermissionError:
        pass  # not root — skip silently
    except Exception:
        pass


def centered_text(draw, y, text, font, fill):
    bbox = draw.textbbox((0, 0), text, font=font)
    tw = bbox[2] - bbox[0]
    draw.text(((W - tw) // 2, y), text, font=font, fill=fill)


# ---------------------------------------------------------------------------
# Pre-build background (expensive, do once)
# ---------------------------------------------------------------------------
bg = make_gradient(W, H)

font_title  = load_font(FONT_BOLD_PATH, 54)
font_sub    = load_font(FONT_PATH,      22)
font_badge  = load_font(FONT_BOLD_PATH, 20)

# A&M badge (bottom-right)
BADGE_TEXT = 'A&M'
BADGE_PAD  = 10
BADGE_R    = 8

# Loading dots cycle
DOT_FRAMES = ['Loading .  ', 'Loading .. ', 'Loading ...']

# ---------------------------------------------------------------------------
# Main animation loop
# ---------------------------------------------------------------------------
start = time.monotonic()
frame = 0
X_SOCK = '/tmp/.X11-unix/X0'

while True:
    elapsed = time.monotonic() - start

    # Exit conditions
    if os.path.exists(X_SOCK):
        break
    if elapsed > 90:
        break

    # Build frame
    img  = bg.copy()
    draw = ImageDraw.Draw(img)

    # Title — vertically centred slightly above middle
    title_y = H // 2 - 70
    centered_text(draw, title_y, 'Welcome to PortaMail', font_title, fill=WHITE)

    # Loading indicator
    dot_text = DOT_FRAMES[frame % len(DOT_FRAMES)]
    centered_text(draw, title_y + 80, dot_text, font_sub, fill=GOLD)

    # Thin progress bar (fills over 15 s then resets)
    BAR_W   = 320
    BAR_H   = 4
    BAR_X   = (W - BAR_W) // 2
    BAR_Y   = title_y + 118
    prog    = min(1.0, (elapsed % 15) / 15)
    draw.rectangle([BAR_X, BAR_Y, BAR_X + BAR_W, BAR_Y + BAR_H], fill=GOLD_DIM)
    draw.rectangle([BAR_X, BAR_Y, BAR_X + int(BAR_W * prog), BAR_Y + BAR_H], fill=GOLD)

    # A&M badge — bottom right
    bx0 = W - 70
    by0 = H - 44
    bx1 = W - 14
    by1 = H - 14
    draw.rounded_rectangle([bx0, by0, bx1, by1], radius=BADGE_R,
                            outline=WHITE, width=2)
    bbox = draw.textbbox((0, 0), BADGE_TEXT, font=font_badge)
    bw = bbox[2] - bbox[0]
    bh = bbox[3] - bbox[1]
    draw.text((bx0 + (bx1 - bx0 - bw) // 2, by0 + (by1 - by0 - bh) // 2 - 1),
              BADGE_TEXT, font=font_badge, fill=WHITE)

    write_fb(image_to_fb_bytes(img))

    frame += 1
    time.sleep(0.5)
