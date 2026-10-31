#!/usr/bin/env python3
"""
make_wheel_decal.py -- printable IR-encoder decal for the 10 in drive wheels.

Matches config.hpp: ENC_STRIPES black bands (and as many white), band ring from
R_IN to R_OUT, IR read radius READ_R. Change the numbers below if you change
config.hpp, then re-run:   python make_wheel_decal.py   (needs: pip install reportlab)

Output: wheel_decal_<N>stripes.pdf
  page 1  instructions
  page 2  half ring A (US Letter, landscape)  } print BOTH, twice (two wheels)
  page 3  half ring B (US Letter, landscape)  }
  page 4  full disc, one piece (11x17 / Tabloid)
"""
import math, sys
from reportlab.lib.pagesizes import letter, landscape, portrait
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.pdfgen import canvas

N = 24            # ENC_STRIPES
R_IN = 4.0        # inner edge of the bands [in]
R_OUT = 4.9       # outer edge of the bands [in]
READ_R = 4.5      # ENC_READ_RADIUS_IN [in]
WHEEL_D = 10.0    # WHEEL_DIAM_IN [in]
HUB_HOLE_D = 1.6  # center cut-out on the one-piece disc (kit hub is 1.5 in max)
TABLOID = (11 * inch, 17 * inch)
RED = colors.HexColor('#c0392b'); GREY = colors.HexColor('#777777')

def band_path(c, cx, cy, a0, a1, r0, r1, n=16):
    p = c.beginPath()
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        x, y = cx + r0 * math.cos(a) * inch, cy + r0 * math.sin(a) * inch
        (p.moveTo if i == 0 else p.lineTo)(x, y)
    for i in range(n + 1):
        a = math.radians(a1 - (a1 - a0) * i / n)
        p.lineTo(cx + r1 * math.cos(a) * inch, cy + r1 * math.sin(a) * inch)
    p.close()
    c.drawPath(p, stroke=0, fill=1)

def bands(c, cx, cy, deg_from, deg_to):
    step = 360.0 / (2 * N)
    c.setFillColor(colors.black)
    k = 0
    while k * step < deg_to - 1e-9:
        a0 = k * step
        if a0 >= deg_from - 1e-9 and k % 2 == 0:
            band_path(c, cx, cy, a0, a0 + step, R_IN, R_OUT)
        k += 1

def scale_square(c, x, y):
    c.setStrokeColor(colors.black); c.setLineWidth(0.6)
    c.rect(x, y, 1 * inch, 1 * inch)
    c.setFont('Helvetica', 7); c.setFillColor(colors.black)
    c.drawString(x, y - 10, 'must measure exactly 1.00 in')

def dashed(c, on=True):
    c.setDash(3, 2) if on else c.setDash()

def half_page(c, label):
    W, H = landscape(letter)
    c.setPageSize((W, H))
    cx, cy = W / 2, 0.75 * inch
    bands(c, cx, cy, 0, 180)
    # cut lines
    c.setStrokeColor(GREY); c.setLineWidth(0.5); dashed(c)
    for r in (R_IN - 0.1, R_OUT + 0.05):
        c.arc(cx - r * inch, cy - r * inch, cx + r * inch, cy + r * inch, 0, 180)
    c.line(cx - (R_OUT + 0.05) * inch, cy, cx - (R_IN - 0.1) * inch, cy)
    c.line(cx + (R_IN - 0.1) * inch, cy, cx + (R_OUT + 0.05) * inch, cy)
    # centering tab: strip from the center up to the ring, cut off after gluing
    tw = 0.5 * inch
    c.rect(cx - tw / 2, cy, tw, (R_IN - 0.1) * inch, stroke=1, fill=0)
    dashed(c, False)
    c.setStrokeColor(colors.black); c.setLineWidth(0.4)
    c.line(cx - 0.3 * inch, cy, cx + 0.3 * inch, cy); c.line(cx, cy - 0.05 * inch, cx, cy + 0.3 * inch)
    c.circle(cx, cy, 0.125 * inch, stroke=1, fill=0)
    c.setFont('Helvetica', 6.5); c.setFillColor(colors.black)
    c.saveState(); c.translate(cx + 0.06 * inch, cy + 0.5 * inch); c.rotate(90)
    c.drawString(0, 0, 'CENTERING TAB: put on axle center, glue ring, then cut tab off')
    c.restoreState()
    # read radius
    c.setStrokeColor(RED); c.setLineWidth(0.6)
    c.arc(cx - READ_R * inch, cy - READ_R * inch, cx + READ_R * inch, cy + READ_R * inch, 0, 180)
    c.setFillColor(RED); c.setFont('Helvetica', 7)
    c.drawString(cx + 0.15 * inch, cy + (READ_R + 0.02) * inch, f'IR read radius {READ_R} in')
    # labels
    c.setFillColor(colors.black); c.setFont('Helvetica-Bold', 11)
    c.drawString(0.6 * inch, H - 0.7 * inch, f'LINKS 701 wheel encoder decal - {label}')
    c.setFont('Helvetica', 8)
    c.drawString(0.6 * inch, H - 0.9 * inch, f'{N} black + {N} white bands, {R_IN}-{R_OUT} in radius, for a {WHEEL_D:g} in wheel. '
                 'Print at 100% / Actual size. Cut on the grey dashed lines.')
    c.drawString(0.6 * inch, H - 1.05 * inch, 'Each wheel needs one A and one B. The straight edges butt together; both ends are band boundaries.')
    scale_square(c, W - 1.6 * inch, H - 1.9 * inch)
    c.showPage()

def full_page(c):
    W, H = TABLOID
    c.setPageSize((W, H))
    cx, cy = W / 2, H / 2 - 0.5 * inch
    bands(c, cx, cy, 0, 360)
    c.setStrokeColor(GREY); c.setLineWidth(0.5); dashed(c)
    c.circle(cx, cy, (R_OUT + 0.05) * inch, stroke=1, fill=0)
    c.circle(cx, cy, HUB_HOLE_D / 2 * inch, stroke=1, fill=0)
    dashed(c, False)
    c.setStrokeColor(colors.black); c.setLineWidth(0.4)
    c.line(cx - 0.2 * inch, cy, cx + 0.2 * inch, cy); c.line(cx, cy - 0.2 * inch, cx, cy + 0.2 * inch)
    c.setStrokeColor(RED); c.setLineWidth(0.6); c.circle(cx, cy, READ_R * inch, stroke=1, fill=0)
    c.setFillColor(colors.black); c.setFont('Helvetica-Bold', 12)
    c.drawString(0.6 * inch, H - 0.7 * inch, 'LINKS 701 wheel encoder decal - one piece (11 x 17 / Tabloid)')
    c.setFont('Helvetica', 8.5)
    c.drawString(0.6 * inch, H - 0.92 * inch, f'{N} black + {N} white bands, {R_IN}-{R_OUT} in radius. Print at 100% on 11x17. '
                 f'Cut the outer dashed circle and the {HUB_HOLE_D} in hub hole.')
    scale_square(c, W - 1.6 * inch, H - 1.9 * inch)
    c.showPage()

def instructions(c):
    W, H = letter
    c.setPageSize((W, H))
    y = H - 0.9 * inch
    c.setFont('Helvetica-Bold', 15); c.drawString(0.8 * inch, y, 'Wheel encoder decal - how to make and fit it')
    lines = [
        (f'Matches config.hpp: ENC_STRIPES = {N}, ENC_READ_RADIUS_IN = {READ_R}, WHEEL_DIAM_IN = {WHEEL_D:g}.', 'Helvetica'),
        (f'Travel per counted edge = pi x {WHEEL_D:g} / (2 x {N}) = {math.pi * WHEEL_D / (2 * N):.3f} in.   '
         f'Band width at the read radius = {math.pi * READ_R / N:.2f} in.', 'Helvetica'),
        ('', 'Helvetica'),
        ('1. PRINT WITH A LASER PRINTER (or pigment/carbon ink).', 'Helvetica-Bold'),
        ('   Many inkjet black inks are dye-based and nearly invisible to infrared: the IR sensor sees "white" everywhere.', 'Helvetica'),
        ('   Laser toner is carbon black and reads as solid black to IR. Test a scrap with the sensor before gluing.', 'Helvetica'),
        ('2. PRINT AT 100% / "Actual size" and measure the 1.00 in square.', 'Helvetica-Bold'),
        ('   Small scale errors do NOT change the distance math (the count per revolution stays 48), but a big', 'Helvetica'),
        ('   shrink can move the bands off the IR read radius.', 'Helvetica'),
        ('3. USE MATTE PAPER, NOT GLOSSY. Do not cover the bands with shiny tape - glare confuses IR sensors.', 'Helvetica-Bold'),
        ('   If you seal it, use matte clear tape or matte spray, then re-test.', 'Helvetica'),
        ('4. FIT IT ON THE INSIDE FACE OF THE WHEEL (the face the sensor looks at).', 'Helvetica-Bold'),
        ('   Letter halves A + B: line the tab cross-hair up on the axle center, glue, cut the tab off, repeat for B.', 'Helvetica'),
        ('   Tabloid one-piece: cut the hub hole, slide it over the hub, glue flat.', 'Helvetica'),
        ('   Spray adhesive or contact cement keeps it flat; bubbles and lifted edges cause missed counts.', 'Helvetica'),
        ('5. CENTERING IS FORGIVING. An off-center decal makes some bands pass faster than others, but every', 'Helvetica-Bold'),
        ('   revolution still has exactly 48 edges, so total distance stays right. Within ~1/16 in is plenty.', 'Helvetica'),
        ('6. CHECK IT (Exercise W3): turn the wheel one revolution by hand - the count must be 48, both wheels,', 'Helvetica-Bold'),
        ('   both directions. Keep the sensor-to-decal gap the same on both sides and shield it from room light.', 'Helvetica'),
        ('', 'Helvetica'),
        ('Rules note: a printed paper decal glued to the team-made wheel was confirmed as allowed (decal ruling).', 'Helvetica-Oblique'),
        ('Do not put decals on returnable kit items. AI-drafted template: cite per the Engineering Notebook rules.', 'Helvetica-Oblique'),
    ]
    y -= 0.4 * inch
    for t, f in lines:
        c.setFont(f, 9.5); c.drawString(0.8 * inch, y, t); y -= 0.24 * inch
    c.showPage()

if __name__ == '__main__':
    out = f'wheel_decal_{N}stripes.pdf' if len(sys.argv) < 2 else sys.argv[1]
    c = canvas.Canvas(out, pagesize=letter)
    c.setTitle(f'LINKS 701 wheel encoder decal, {N} stripes')
    instructions(c)
    half_page(c, 'half A')
    half_page(c, 'half B')
    full_page(c)
    c.save()
    print('wrote', out)
