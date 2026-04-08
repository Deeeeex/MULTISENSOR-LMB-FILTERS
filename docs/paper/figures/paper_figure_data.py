from __future__ import annotations

from pathlib import Path
import csv


def get_scalar_figure_data() -> dict:
    return {
        "figure3": {
            "r": [i / 200.0 for i in range(201)],
            "existence_confidence_min_score": 0.85,
            "existence_confidence_power": 2.0,
        },
        "figure5": {
            "title": "Factor Ablation Under Tiered Packet Loss",
            "arms": [
                "fixed",
                "+covariance",
                "+link quality",
                "+existence confidence",
                "+structure-aware decoupled KLA",
            ],
            "metrics": {
                "Consensus OSPA": [2.624065, 2.211513, 1.877771, 1.874840, 1.862244],
                "Consensus RMSE": [2.702602, 2.410976, 1.800945, 1.779820, 1.749608],
                "Consensus Card": [0.878750, 0.589500, 0.245250, 0.244500, 0.244250],
            },
            "sources": [
                "RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md",
                "RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md",
            ],
        },
        "figure6": {
            "title": "Ideal-Communication Supporting Comparison",
            "consensus": {
                "labels": ["OSPA", "RMSE", "Card"],
                "ga": [1.705549, 1.525900, 0.160500],
                "adaptive": [1.494474, 1.289643, 0.139000],
            },
            "local": {
                "labels": ["E-OSPA", "RMSE"],
                "ga": [1.949511, 1.441872],
                "adaptive": [1.876801, 1.369361],
            },
            "sources": [
                "RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md",
            ],
        },
    }


def load_figure4_series(csv_path: str | Path) -> dict:
    path = Path(csv_path)
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)

    return {
        "time": [float(row["time"]) for row in rows],
        "ospa_fixed": [float(row["ospa_fixed"]) for row in rows],
        "ospa_adaptive": [float(row["ospa_adaptive"]) for row in rows],
        "rmse_fixed": [float(row["rmse_fixed"]) for row in rows],
        "rmse_adaptive": [float(row["rmse_adaptive"]) for row in rows],
        "card_fixed": [float(row["card_fixed"]) for row in rows],
        "card_adaptive": [float(row["card_adaptive"]) for row in rows],
    }
