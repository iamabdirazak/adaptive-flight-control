"""The PID baseline flown through every scenario, the numbers to beat.

    python -m src.evaluation.baseline
"""
import json

import matplotlib.pyplot as plt
import torch

from ..control.cascade import Cascade
from ..paths import FIGURES, REPORTS
from ..plant.vehicle import Vehicle
from ..visualization import style
from .metrics import measure
from .rollout import rollout
from .scenarios import SCENARIOS

TRIALS = 200


def study(make_controller, vehicle, trials=TRIALS):
    results = {}
    for s in SCENARIOS:
        values = measure(s.metric, rollout(s, make_controller, vehicle, trials, seed=int(s.key[1:])))
        finite = values[torch.isfinite(values)]
        results[s.key] = dict(
            name=s.name, metric=s.metric, unit=s.unit, target=s.target,
            median=values.median().item(), p95=values.quantile(0.95).item() if finite.numel() == values.numel() else float("inf"),
            passed=(values <= s.target).float().mean().item(),
        )
        r = results[s.key]
        print(f"{s.key} {s.name:<18} median {r['median']:.3f} {s.unit}  pass {r['passed']:.0%}")
    return results


def figure(results, name="baseline"):
    style.apply()
    fig, axis = plt.subplots(figsize=(7.6, 2.8))
    keys = list(results)
    rates = [100 * results[k]["passed"] for k in keys]
    axis.set_xlim(-0.6, len(keys) - 0.4)
    axis.set_ylim(0, 100)
    axis.set_xticks(range(len(keys)), [f"{k}\n" + results[k]['name'].replace(' ', '\n', 1) for k in keys])
    axis.set_ylabel("Trials passed (%)")
    axis.set_title("PID baseline")
    style.rounded_bars(axis, range(len(keys)), [max(r, 0.5) for r in rates], [style.GREY] * len(keys))
    for x, r in enumerate(rates):
        axis.text(x, r + 3, f"{r:.0f}%", ha="center", va="bottom", fontsize=8, color=style.NEUTRAL["ink"])
    axis.set_ylim(0, 112)
    axis.set_yticks([0, 25, 50, 75, 100])
    style.save(fig, FIGURES, name)


def main():
    vehicle = Vehicle()
    results = study(lambda n: Cascade(vehicle, n), vehicle)
    REPORTS.mkdir(exist_ok=True)
    (REPORTS / "baseline.json").write_text(json.dumps(results, indent=2))
    figure(results)


if __name__ == "__main__":
    main()
