# make_decal_sheet.py -- LINKS 701 forearm decal sheet (Letter, 100 %). Layouts match config.hpp DECAL_LEFT / DECAL_RIGHT.
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, black, white
inch = 72.0
C = {'C': HexColor('#00E5FF'), 'M': HexColor('#FF00A0'), 'O': HexColor('#FF6600'), 'V': HexColor('#7F00FF')}
SQ, G = 0.85, 0.10           # square, gutter/border (in)
def decal(c, x, y, tl, tr, bl, br, title, left_lbl, right_lbl):
    """x, y = lower-left of the 2.0 in decal (in)."""
    c.setFillColor(black); c.rect(x*inch, y*inch, 2.0*inch, 2.0*inch, stroke=0, fill=1)
    for (col, dx, dy) in ((tl, 0, 1), (tr, 1, 1), (bl, 0, 0), (br, 1, 0)):
        c.setFillColor(C[col]); c.rect((x+G+dx*(SQ+G))*inch, (y+G+dy*(SQ+G))*inch, SQ*inch, SQ*inch, stroke=0, fill=1)
    # cut marks just outside the corners (the cut line IS the black edge)
    c.setStrokeColor(black); c.setLineWidth(0.4)
    for cx, cy in ((x, y), (x+2, y), (x, y+2), (x+2, y+2)):
        sx = -1 if cx == x else 1; sy = -1 if cy == y else 1
        c.line((cx+sx*0.05)*inch, cy*inch, (cx+sx*0.25)*inch, cy*inch)
        c.line(cx*inch, (cy+sy*0.05)*inch, cx*inch, (cy+sy*0.25)*inch)
    # centre ticks (line the decal centre up 9.0 in from the axle-2 centre)
    c.line((x+1)*inch, (y-0.05)*inch, (x+1)*inch, (y-0.18)*inch); c.line((x+1)*inch, (y+2.05)*inch, (x+1)*inch, (y+2.18)*inch)
    c.setFillColor(black); c.setFont('Helvetica-Bold', 8.5); c.drawCentredString((x+1)*inch, (y+2.32)*inch, title)
    c.setFont('Helvetica', 7); c.drawCentredString((x+1)*inch, (y+2.21)*inch, 'TOP EDGE (up when the arm points forward, level)')
    c.saveState(); c.translate((x-0.12)*inch, (y+1)*inch); c.rotate(90); c.drawCentredString(0, 0, left_lbl); c.restoreState()
    c.saveState(); c.translate((x+2.12)*inch, (y+1)*inch); c.rotate(-90); c.drawCentredString(0, 0, right_lbl); c.restoreState()
c = canvas.Canvas('LINKS_forearm_decal_letter.pdf', pagesize=letter)
c.setTitle('LINKS 701 forearm camera self-check decals')
c.setFont('Helvetica-Bold', 13); c.drawString(0.6*inch, 10.45*inch, 'LINKS 701 forearm decals - camera self-check (Byte to Bite 2026)')
c.setFont('Helvetica', 8.5)
lines = ['Print at 100 % / Actual size on a COLOUR LASER printer, MATTE paper. Check the 1.00 in square. Cut on the black edge (2.00 in).',
         'One decal on EACH side face of ForeArm V2, centred across the 2.0 in bar, decal centre 9.0 in from the AXLE-2 centre (spans 8.0-10.0 in).',
         'LEFT-FACE decal on the robot-left face, RIGHT-FACE decal on the robot-right face. Match the AXLE 2 / AXLE 3 END labels to the forearm.',
         'Glue flat (spray adhesive / contact cement); no gloss tape over it. Not on returnable kit items.',
         'Colours (hub-confirmed): CYAN #00E5FF  MAGENTA #FF00A0  ORANGE #FF6600  VIOLET #7F00FF.',
         'Teach the colours from THIS print, under real lighting, in the AI Vision Utility.',
         'Printed paper glued on = decal (hub). Colour used functionally: rule 1.3.2.3.B. AI-drafted template: cite per the Engineering Notebook rules.']
for i, t in enumerate(lines): c.drawString(0.6*inch, (10.2 - 0.15*i)*inch, t)
c.setFont('Helvetica', 8.5)
# LEFT face: axle 3 appears on the LEFT when you look at the robot-left face.  arm terms: a2/top C, a3/top M, a2/bot O, a3/bot V
for i, x in enumerate((0.9, 3.8)):
    decal(c, x, 6.2, 'M', 'C', 'V', 'O', 'LEFT-FACE decal' + (' (spare)' if i else ''), 'AXLE 3 END (FlapJack)', 'AXLE 2 END (gear)')
# RIGHT face: axle 3 appears on the RIGHT.  arm terms: a2/top V, a3/top O, a2/bot M, a3/bot C
for i, x in enumerate((0.9, 3.8)):
    decal(c, x, 2.9, 'V', 'O', 'M', 'C', 'RIGHT-FACE decal' + (' (spare)' if i else ''), 'AXLE 2 END (gear)', 'AXLE 3 END (FlapJack)')
c.setLineWidth(0.6); c.rect(6.6*inch, 6.8*inch, 1*inch, 1*inch)
c.setFont('Helvetica', 7.5); c.drawCentredString(7.1*inch, 6.62*inch, 'must measure exactly 1.00 in')
c.setFont('Helvetica', 7.5)
notes = ['Mirror layouts: the camera tells', 'the faces apart by handedness,', 'so a wrong CAM_SIDE is caught.', '',
         'Self-check pose: arm level', '(pot 0 deg) = B+Dup PLATE CARRY.', '',
         'Squares 0.85 in, gutter and', 'border 0.10 in, pitch 0.95 in', '(config.hpp DECAL_PITCH_IN).']
for i, t in enumerate(notes): c.drawString(6.45*inch, (6.2 - 0.14*i)*inch, t)
c.showPage(); c.save()
print('ok')
