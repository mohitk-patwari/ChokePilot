"""
Open-loop step-test analysis and dynamic model identification.

Runs a fresh choke staircase against data/simulator.py's Simulator -- per the
challenge brief, "students are expected to generate their own data using the
simulator and develop their control-oriented models from these experiments," so this
does not reuse the reference CSV directly. It applies the same identification method
used to calibrate the simulator (saturating steady-state map + FOPDT dynamics, fit by
brute-force grid search + closed-form regression) to this fresh experiment, producing
the MODEL that controller.py's brute-force MPC uses for its internal prediction.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import Simulator, TS_HOURS, _fit_fopdt, _simulate_fopdt  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)

# Monotonic staircase across the full 0-100% range: a naturally flowing well with no
# artificial lift and no changing reservoir properties (challenge's own assumptions)
# isn't expected to show hysteresis, so an up-only sweep is enough to characterize it.
STEP_LEVELS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
DWELL_HOURS = 24  # ~4x the slowest calibrated time constant + dead time -> near-settled
CHANNELS = ["Q", "WHP", "FLP", "BHP"]


def run_step_test(seed=0):
    sim = Simulator(initial_choke=STEP_LEVELS[0], seed=seed)
    rows = []
    t = 0
    for level in STEP_LEVELS:
        for _ in range(DWELL_HOURS):
            q, whp, flp, bhp = sim.step(level)
            rows.append((t, level, q, whp, flp, bhp))
            t += 1
    df = pd.DataFrame(rows, columns=["Time_hr", "Choke_pct", "Q", "WHP", "FLP", "BHP"])
    df.to_csv(OUTPUT_DIR / "step_test_data.csv", index=False)
    return df


def plot_step_test(df):
    fig, axes = plt.subplots(5, 1, figsize=(11, 13), sharex=True)
    axes[0].step(df.Time_hr, df.Choke_pct, where="post", color="black")
    axes[0].set_ylabel("Choke (%)")
    axes[0].set_title("Open-loop step test: response to choke staircase")
    labels = {"Q": "Oil Rate (bbl/hr)", "WHP": "WHP (psi)", "FLP": "FLP (psi)", "BHP": "BHP (psi)"}
    for ax, ch in zip(axes[1:], CHANNELS):
        ax.plot(df.Time_hr, df[ch])
        ax.set_ylabel(labels[ch])
    axes[-1].set_xlabel("Time (hr)")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "step_test_response.png", dpi=130)
    plt.close(fig)
    print(f"saved {OUTPUT_DIR / 'step_test_response.png'}")


def identify_model(df):
    u = df["Choke_pct"].to_numpy(dtype=float)
    model = {}
    for ch in CHANNELS:
        y = df[ch].to_numpy(dtype=float)
        model[ch] = _fit_fopdt(u, y, TS_HOURS, clip_nonneg=(ch == "Q"))
    return model


def demo():
    """Self-check: identify a model from a fresh step test and confirm it reproduces
    that same step test (the model has to explain the data it was fit to)."""
    df = run_step_test(seed=1)
    model = identify_model(df)
    u = df["Choke_pct"].to_numpy(dtype=float)
    for ch, p in model.items():
        y = df[ch].to_numpy(dtype=float)
        sim = _simulate_fopdt(u, y[0], p["A"], p["B"], p["uh"], p["tau"], p["theta_steps"],
                               TS_HOURS, clip_nonneg=p["clip_nonneg"])
        rmse = float(np.sqrt(np.mean((sim - y) ** 2)))
        band = y.max() - y.min()
        assert rmse < 0.15 * band, f"{ch}: RMSE {rmse:.2f} too high vs data band {band:.2f}"
        print(f"{ch}: gain(u=100)-gain(u=0)={p['B']:.2f}  tau={p['tau']:.2f}h  "
              f"theta={p['theta_steps']}h  RMSE={rmse:.2f} (band={band:.1f})")
    print("identify.py self-check PASSED")


if __name__ == "__main__":
    df = run_step_test()
    plot_step_test(df)
    model = identify_model(df)
    for ch, p in model.items():
        print(f"{ch}: A={p['A']:.2f} B={p['B']:.2f} uh={p['uh']:.1f} tau={p['tau']:.2f}h "
              f"theta={p['theta_steps']}h noise_std={p['noise_std']:.2f}")
    print()
    demo()
