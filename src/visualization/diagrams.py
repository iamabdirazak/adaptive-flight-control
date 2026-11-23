"""Documentation diagrams: the controller as a file, and its architecture.

The layout: a soft frame with a title, a symmetric row of raised
cards, each with an icon, a name and a short description, and small arrows
between them.
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, Polygon

from src.paths import ROOT
from src.visualization import style

DOCS = ROOT / "docs"
AQUA, SKY, VIOLET = (style.SHADES[k] for k in ("aqua", "sky", "violet"))
LIGHT, DEEP = "#BFE3FF", "#1F5C99"
FRAME, CARD, ARROW = style.NEUTRAL["background"], style.NEUTRAL["surface"], style.NEUTRAL["tertiary"]

CARD_W, CARD_H, GAP = 2.6, 3.4, 0.41  # portrait cards, like the UAV figure
MARGIN = 0.4  # around the row of cards, equal to CARD_Y
CARD_Y = 0.4
CARD_RADIUS, FRAME_RADIUS = 0.27, 0.38  # in proportion to the UAV figure's cards and frame
HEIGHT = CARD_Y + CARD_H + 1.0  # room above the cards for the title
ICON_Y = CARD_Y + 2.25
NAME_Y, SUBTITLE_Y = CARD_Y + 1.05, CARD_Y + 0.65
MIDDLE = CARD_Y + CARD_H / 2


# Drawing primitives ---------------------------------------------------------

def rounded(axis, x, y, w, h, color, radius=0.12, z=3):
    axis.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={radius}",
                                  facecolor=color, edgecolor="none", zorder=z))


def wave(axis, cx, cy, width, amplitude, color, cycles=1.0, lw=2.2, z=4):
    """A smooth rise, as a sensor trace draws it."""
    x = np.linspace(0, 1, 120)
    y = 10 * x**3 - 15 * x**4 + 6 * x**5
    axis.plot(cx - width / 2 + x * width, cy - amplitude + 2 * amplitude * y, color=color, linewidth=lw,
              solid_capstyle="round", zorder=z)


def dot(axis, x, y, r, color, alpha=1.0, z=5):
    axis.add_patch(Circle((x, y), r, facecolor=color, edgecolor="none", alpha=alpha, zorder=z))


# Icons, each centred on (cx, cy) ------------------------------------------

INK = style.NEUTRAL["ink"]


def outline(axis, x, y, w, h, color, radius=0.1, lw=1.6, z=3, dash=None):
    axis.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={radius}", facecolor="none",
                                  edgecolor=color, linewidth=lw, linestyle=dash or "-", zorder=z))


def link(axis, a, b, color, width=9, z=4, alpha=1.0):
    axis.plot([a[0], b[0]], [a[1], b[1]], color=color, linewidth=width, solid_capstyle="round", alpha=alpha, zorder=z)


def arrow(axis, x, y, dx, dy, color, z=6, head=0.13):
    """A short arrow from (x, y) along (dx, dy), with a filled head."""
    length = np.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    end = (x + dx - ux * head, y + dy - uy * head)
    link(axis, (x, y), end, color, 2.4, z)
    tip = (x + dx, y + dy)
    axis.add_patch(Polygon([tip, (end[0] - uy * head * 0.7, end[1] + ux * head * 0.7),
                            (end[0] + uy * head * 0.7, end[1] - ux * head * 0.7)],
                           facecolor=color, edgecolor="none", zorder=z))


def icon_input(axis, cx, cy):
    """State and motor RPM arriving frame after frame, each holding a sensor trace."""
    for offset, alpha in ((-0.28, 0.3), (-0.14, 0.55), (0.0, 1.0)):
        rounded(axis, cx - 0.5 + offset, cy - 0.34 - offset, 0.95, 0.68, DEEP, 0.12, z=3 + alpha)
        axis.patches[-1].set_alpha(alpha)
    wave(axis, cx - 0.02, cy, 0.6, 0.18, LIGHT, lw=2.4, z=6)


def icon_weight(axis, cx, cy):
    """The body between the thrust that holds it and the weight that pulls it down."""
    rounded(axis, cx - 0.34, cy - 0.2, 0.68, 0.4, SKY, 0.1, z=4)
    arrow(axis, cx, cy + 0.24, 0, 0.42, AQUA)
    arrow(axis, cx, cy - 0.24, 0, -0.42, VIOLET)


def icon_dynamics(axis, cx, cy):
    """A motor's command and the slower speed that follows it."""
    x0, w, lo, hi = cx - 0.6, 1.2, cy - 0.38, cy + 0.34
    axis.plot([x0, x0 + 0.25, x0 + 0.25, x0 + w], [lo, lo, hi, hi], color=DEEP, linewidth=1.8,
              linestyle=(0, (2, 2)), zorder=4)                      # the command
    t = np.linspace(0, 1, 80)
    axis.plot(x0 + 0.25 + t * (w - 0.25), lo + (hi - lo) * (1 - np.exp(-t * 4.2)), color=LIGHT, linewidth=2.4,
              solid_capstyle="round", zorder=5)                     # the speed, lagging
    axis.plot([x0, x0 + 0.25], [lo, lo], color=LIGHT, linewidth=2.4, solid_capstyle="round", zorder=5)


def icon_health(axis, cx, cy):
    """Four rotors on an X frame, one of them down to part of its thrust."""
    d = 0.4
    for sx, sy in ((1, 1), (-1, -1)):
        link(axis, (cx - sx * d, cy - sy * d), (cx + sx * d, cy + sy * d), DEEP, 6, z=3)
    link(axis, (cx - d, cy + d), (cx + d, cy - d), DEEP, 6, z=3)
    for sx, sy, color in ((-1, 1, AQUA), (1, 1, SKY), (-1, -1, SKY), (1, -1, VIOLET)):
        x, y = cx + sx * d, cy + sy * d
        if color is VIOLET:
            a = np.radians(np.linspace(90, 90 - 0.3 * 360, 40))      # a weak motor: a third of its ring
            axis.add_patch(Circle((x, y), 0.22, facecolor="none", edgecolor=VIOLET, linewidth=1.4,
                                  alpha=0.3, zorder=4))
            axis.plot(x + 0.22 * np.cos(a), y + 0.22 * np.sin(a), color=VIOLET, linewidth=2.6,
                      solid_capstyle="round", zorder=5)
        else:
            axis.add_patch(Circle((x, y), 0.22, facecolor="none", edgecolor=color, linewidth=2.6, zorder=5))
        dot(axis, x, y, 0.06, LIGHT, z=6)


def icon_cascade(axis, cx, cy):
    """Loops nested inside each other: position, velocity, attitude, rate."""
    for i, color in enumerate((DEEP, SKY, AQUA, LIGHT)):
        r = 0.58 - 0.14 * i
        outline(axis, cx - r, cy - r * 0.72, 2 * r, 1.44 * r, color, radius=0.1 + 0.02 * (3 - i), lw=2.0, z=4 + i)
    dot(axis, cx, cy, 0.07, LIGHT, z=9)


def icon_output(axis, cx, cy):
    """Four motor commands, one per rotor, sent 500 times a second."""
    heights = (0.72, 0.62, 0.8, 0.5)
    for i, (h, color) in enumerate(zip(heights, (AQUA, SKY, SKY, VIOLET))):
        rounded(axis, cx - 0.5 + i * 0.27, cy - 0.45, 0.18, h, color, 0.06, z=4)
    axis.plot([cx - 0.6, cx + 0.6], [cy - 0.47, cy - 0.47], color=style.NEUTRAL["tertiary"], linewidth=1, zorder=3)


def diagram(title: str, cards: list[tuple], name: str) -> None:
    style.apply()
    width = len(cards) * CARD_W + (len(cards) - 1) * GAP + 2 * MARGIN
    figure, axis = plt.subplots(figsize=(width / 1.45, HEIGHT / 1.45))
    axis.set_xlim(0, width)
    axis.set_ylim(0, HEIGHT)
    axis.set_aspect("equal")
    axis.axis("off")

    rounded(axis, 0, 0, width, HEIGHT, FRAME, FRAME_RADIUS, z=1)
    axis.text(width / 2, HEIGHT - 0.5, title, ha="center", va="center", fontsize=13,
              fontweight="medium", color=style.NEUTRAL["ink"], zorder=6)

    for index, (name_text, subtitle, draw) in enumerate(cards):
        x = MARGIN + index * (CARD_W + GAP)
        centre = x + CARD_W / 2
        rounded(axis, x, CARD_Y, CARD_W, CARD_H, CARD, CARD_RADIUS, z=2)
        draw(axis, centre, ICON_Y)
        axis.text(centre, NAME_Y, name_text, ha="center", va="center", fontsize=10,
                  fontweight="medium", color=style.NEUTRAL["ink"], zorder=6)
        axis.text(centre, SUBTITLE_Y, subtitle, ha="center", va="center", fontsize=7.4,
                  color=style.NEUTRAL["secondary"], zorder=6)
        if index:
            mid = x - GAP / 2
            axis.add_patch(Polygon([[mid - 0.07, MIDDLE - 0.12], [mid - 0.07, MIDDLE + 0.12], [mid + 0.09, MIDDLE]],
                                   facecolor=ARROW, edgecolor="none", zorder=3))

    style.save(figure, DOCS, name, panel=False)


def architecture() -> None:
    diagram("Controller Architecture", [
        ("Input", "State and motor RPM", icon_input),
        ("Weight", "Hover thrust", icon_weight),
        ("Dynamics", "Motor lag and inertia", icon_dynamics),
        ("Motor Health", "Missing torque", icon_health),
        ("Cascade", "Adaptive PID, 500 Hz", icon_cascade),
        ("Output", "Motor commands", icon_output),
    ], "architecture")


# The model as a file -------------------------------------------------------

from matplotlib.colors import LinearSegmentedColormap
from matplotlib.path import Path as Shape
from matplotlib.patches import PathPatch


def fill(axis, patch, colors, angle=90, z=3):
    """Fill a patch with a linear gradient; angle 90 runs bottom to top."""
    axis.add_patch(patch)
    patch.set_facecolor("none")
    patch.set_edgecolor("none")
    x0, y0, x1, y1 = patch.get_path().transformed(patch.get_patch_transform()).get_extents().bounds
    x1, y1 = x0 + x1, y0 + y1
    t = np.radians(angle)
    gx, gy = np.meshgrid(np.linspace(0, 1, 256), np.linspace(0, 1, 256))
    ramp = gx * np.cos(t) + gy * np.sin(t)
    ramp = (ramp - ramp.min()) / (ramp.max() - ramp.min())
    image = axis.imshow(ramp, extent=(x0, x1, y0, y1), origin="lower", zorder=z,
                        cmap=LinearSegmentedColormap.from_list("", colors), interpolation="bicubic")
    image.set_clip_path(patch)


def corner(points, centre, radius, start, end):
    angles = np.linspace(np.radians(start), np.radians(end), 12)
    points += [(centre[0] + radius * np.cos(a), centre[1] + radius * np.sin(a)) for a in angles]


def page(x, y, w, h, fold, radius):
    """A document outline with rounded corners and a folded top-right corner."""
    points = []
    corner(points, (x + radius, y + radius), radius, 180, 270)
    corner(points, (x + w - radius, y + radius), radius, 270, 360)
    points += [(x + w, y + h - fold), (x + w - fold, y + h)]
    corner(points, (x + radius, y + h - radius), radius, 90, 180)
    return PathPatch(Shape(points + [points[0]], closed=True))


def squircle(x, y, size, radius):
    return FancyBboxPatch((x, y), size, size, boxstyle=f"round,pad=0,rounding_size={radius}")


CAPTION = "adaptive.py  \u00b7  cascaded PID with three estimators  \u00b7  500 Hz"


def controller_file() -> None:
    """The released controller shown as what it is: a single source file."""
    style.apply()
    width, height = 6 * CARD_W + 5 * GAP + 2 * MARGIN, 7.2
    figure, axis = plt.subplots(figsize=(width / 1.45, height / 1.45))
    axis.set_xlim(0, width)
    axis.set_ylim(0, height)
    axis.set_aspect("equal")
    axis.axis("off")
    rounded(axis, 0, 0, width, height, FRAME, FRAME_RADIUS, z=1)

    w, h, fold = 2.55, 3.4, 0.62                                              # document proportions, 3:4
    x, y = width / 2 - w / 2, 2.5
    cx = width / 2

    for spread, alpha in ((0.32, 0.04), (0.2, 0.06), (0.1, 0.09), (0.03, 0.14)):   # soft lift
        shadow = page(x - spread, y - spread - 0.12, w + 2 * spread, h + 2 * spread, fold + spread, 0.2 + spread)
        shadow.set(facecolor="black", edgecolor="none", alpha=alpha, zorder=2)
        axis.add_patch(shadow)

    sheet = page(x, y, w, h, fold, 0.2)
    fill(axis, sheet, ["#1D1D20", "#2A2A2E", "#38383D"], z=3)                # lit from above
    rim = page(x, y, w, h, fold, 0.2)
    rim.set(facecolor="none", edgecolor="#4A4A50", linewidth=1, zorder=4)
    axis.add_patch(rim)
    flap = PathPatch(Shape([(x + w - fold, y + h), (x + w - fold, y + h - fold + 0.1),
                            (x + w - fold + 0.1, y + h - fold), (x + w, y + h - fold),
                            (x + w - fold, y + h)], closed=True))
    fill(axis, flap, ["#3E3E44", "#5A5A61"], angle=45, z=4)

    # The mark: a quadrotor from above, arms in the accent and rotors in its three shades
    my = y + h * 0.6
    d = 0.42
    for (ax_, ay_), (bx, by) in (((-d, -d), (d, d)), ((-d, d), (d, -d))):
        link(axis, (cx + ax_, my + ay_), (cx + bx, my + by), DEEP, 7, z=5)
    rounded(axis, cx - 0.17, my - 0.17, 0.34, 0.34, SKY, 0.08, z=6)
    for (sx, sy), color in zip(((-1, 1), (1, 1), (-1, -1), (1, -1)), (AQUA, SKY, SKY, VIOLET)):
        axis.add_patch(Circle((cx + sx * d, my + sy * d), 0.26, facecolor=color, edgecolor="none",
                              alpha=0.25, zorder=5))
        axis.add_patch(Circle((cx + sx * d, my + sy * d), 0.26, facecolor="none", edgecolor=color,
                              linewidth=2.2, zorder=6))
        dot(axis, cx + sx * d, my + sy * d, 0.06, LIGHT, z=7)
    axis.text(cx, y + h * 0.22, "Python", ha="center", va="center", fontsize=15,
              fontweight="medium", color="#8E8E93", alpha=0.85, zorder=6)    # quieter than the mark

    axis.text(width / 2, 1.7, "Adaptive Flight Controller", ha="center", va="center", fontsize=14,
              fontweight="semibold", color=style.NEUTRAL["ink"], zorder=6)
    axis.text(width / 2, 1.25, CAPTION, ha="center", va="center", fontsize=9,
              color=style.NEUTRAL["secondary"], zorder=6)

    style.save(figure, DOCS, "model", panel=False)


if __name__ == "__main__":
    controller_file()
    architecture()
