from __future__ import annotations

from pathlib import Path
import csv


def get_scalar_figure_data() -> dict:
    return {
        "figure3": {
            "r": [i / 200.0 for i in range(201)],
            "existence_confidence_min_score": 0.85,
            "existence_confidence_power": 2.0,
            "profiles": [
                {
                    "label": "Decisive tails",
                    "r_values": [0.96, 0.91, 0.87, 0.82],
                },
                {
                    "label": "Mixed evidence",
                    "r_values": [0.93, 0.74, 0.58, 0.46],
                },
                {
                    "label": "Ambiguous set",
                    "r_values": [0.57, 0.54, 0.51, 0.47],
                },
            ],
        },
        "figure5": {
            "title": "Factor Ablation Under Tiered Packet Loss",
            "arms": [
                "Fixed Metropolis",
                "Covariance only",
                "Covariance and link quality",
                "Three-factor backbone",
                "Branch-decoupled backbone",
            ],
            "metrics": {
                "Consensus OSPA": [2.447978, 2.082445, 1.829718, 1.829831, 1.821448],
                "Consensus RMSE": [2.583880, 2.167710, 1.764180, 1.753626, 1.749812],
                "Consensus Card": [0.679437, 0.457937, 0.230687, 0.230500, 0.231625],
            },
            "sources": [
                "RUN/GA/GA_TIERED_LINK_ABLATION_MAIN20_20260507.md",
            ],
        },
        "figure6": {
            "title": "Ideal-Communication Supporting Comparison",
            "consensus": {
                "labels": ["OSPA", "RMSE", "Card"],
                "ga": [1.703504, 1.532487, 0.161500],
                "adaptive": [1.481747, 1.238241, 0.132187],
            },
            "local": {
                "labels": ["E-OSPA", "RMSE"],
                "ga": [1.962844, 1.444517],
                "adaptive": [1.885250, 1.371501],
            },
            "sources": [
                "RUN/GA/GA_IDEAL_COMM_MAIN20_PAIRED_20260507.md",
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
