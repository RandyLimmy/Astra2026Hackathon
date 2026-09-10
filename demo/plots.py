"""Plots of computed development results, not an agent-success presentation."""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_development(bundles: list[dict], output_path: Path):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.0), layout="constrained")
    fig.set_facecolor("#f5f7fa")
    for ax in axes:
        ax.set_facecolor("#f5f7fa")
        ax.grid(axis="y", color="#dce2e8", zorder=0)
    selected = next((b for b in bundles if b["case_id"] == "repeated"), bundles[0])
    names = [("reference", "Synthetic reference", "#e26b55"),
             ("candidate", "Original model", "#397cba"),
             ("parameter_fit", "Parameter fit", "#be9235"),
             ("developer_check", "Developer stateful check", "#32917c")]
    for key, label, color in names:
        if key in selected:
            rows = selected[key]["probe"]
            axes[0].plot([r["t_s"] for r in rows], [r["v_mps"] for r in rows],
                         label=label, color=color, linewidth=2.5,
                         linestyle="--" if key == "developer_check" else "-")
    axes[0].set(title="After repeated braking", xlabel="Probe time (seconds)", ylabel="Speed (m/s)")
    axes[0].legend(frameon=False, fontsize=9)
    cases = [b for b in bundles if b["case_id"] != "normal_control"]
    available = [(k, label, c) for k, label, c in names if k in cases[0]]
    x = np.arange(len(cases))
    width = 0.8 / len(available)
    for index, (key, label, color) in enumerate(available):
        values = [b[key]["summary"]["stopping_distance_m"] for b in cases]
        # Missing stops are displayed explicitly rather than treated as zero.
        bars = axes[1].bar(x - 0.4 + width / 2 + index * width,
                          [float("nan") if v is None else v for v in values],
                          width=width, color=color, label=label, zorder=3)
        axes[1].bar_label(bars, labels=["no stop" if v is None else f"{v:.1f}" for v in values],
                          padding=3, fontsize=8)
    axes[1].axhline(selected["config"]["wall_distance_m"], color="#7c8694", linestyle=":", linewidth=1.5)
    axes[1].set(xticks=x, xticklabels=[b["title"] for b in cases], ylabel="Stopping distance (metres)",
                title="Same starting speed and brake command")
    fig.suptitle("RealityPatch · synthetic MuJoCo development experiment", fontsize=17, fontweight="bold")
    fig.text(0.01, -0.035, "Developer check is manually authored. No Astra repair or held-out evaluation has run yet.",
             fontsize=10, color="#485564")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
