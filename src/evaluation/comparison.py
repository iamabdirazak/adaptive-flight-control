"""The fair PID and the adaptive controller side by side, scenario by scenario.

Reads reports/baseline.json and reports/adaptive.json.

    python -m src.evaluation.comparison
"""
import json

import matplotlib.pyplot as plt

from ..paths import FIGURES, REPORTS
from ..visualization import style

WIDTH = 0.36


def main():
    baseline = json.loads((REPORTS / "baseline.json").read_text())
    adaptive = json.loads((REPORTS / "adaptive.json").read_text())
    keys = list(baseline)

    style.apply()
    fig, axis = plt.subplots(figsize=(7.6, 2.9))
    axis.set_xlim(-0.6, len(keys) - 0.4)
    axis.set_ylim(0, 112)
    axis.set_yticks([0, 25, 50, 75, 100])
    axis.set_xticks(range(len(keys)), [f"{k}\n" + baseline[k]["name"].replace(" ", "\n", 1) for k in keys])
    axis.set_ylabel("Trials passed (%)")
    axis.set_title("PID and adaptive")
    for offset, results, color in ((-WIDTH / 2, baseline, style.GREY), (WIDTH / 2, adaptive, style.ACCENT)):
        rates = [100 * results[k]["passed"] for k in keys]
        positions = [x + offset for x in range(len(keys))]
        style.rounded_bars(axis, positions, [max(r, 0.5) for r in rates], [color] * len(keys),
                           width=WIDTH * 0.9, radius=0.06)
        for x, r in zip(positions, rates):
            axis.text(x, r + 3, f"{r:.0f}", ha="center", va="bottom", fontsize=7, color=style.NEUTRAL["ink"])
    style.legend(fig, [("swatch", style.GREY, "Fair PID"), ("swatch", style.ACCENT, "Adaptive")], anchor=(0.97, 0.9))
    style.save(fig, FIGURES, "comparison")


if __name__ == "__main__":
    main()
