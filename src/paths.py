"""Where every file lives, and the rates everything runs at."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
RUNS = ROOT / "runs"
MODELS = ROOT / "models"
REPORTS = ROOT / "reports"
FIGURES = REPORTS / "figures"

CONTROL_PERIOD = 0.002  # 500 Hz, the controller's tick
SIMULATION_STEP = 0.0005  # 2 kHz, the physics, four steps per tick
