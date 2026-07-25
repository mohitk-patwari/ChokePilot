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
from simulator import (  # noqa: E402
    Simulator, TS_HOURS, _fit_fopdt, _simulate_fopdt, steady_state_from_params,
)

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)

# Monotonic staircase across the full 0-100% range: a naturally flowing well with no
# artificial lift and no changing reservoir properties (challenge's own assumptions)
# isn't expected to show hysteresis, so an up-only sweep is enough to characterize it.
STEP_LEVELS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# ~4x the slowest TRUE time constant + dead time -> near-settled. BHP's true tau=9h,
# theta=2h needs ~4*9+2=38h; the previous value of 24h under-dwelled it (and, to a
# lesser extent, WHP/FLP) and was the dominant cause of their large mean tau
# identification error (see verify_identification.py). Confirmed by direct
# comparison: at 24h dwell, mean tau error was Q 20% / WHP 16% / FLP 27% / BHP 31%;
# at 40h, WHP/FLP/BHP drop to roughly 4% / 14% / 8% -- Q alone stays near 19% at
# every dwell tested (24-80h), so its error is NOT dwell-limited; it's a separate,
# still-unexplained identification bias (tests/test_identification.py's thresholds
# reflect this, not a single uniform bound).
DWELL_HOURS = 40
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


def _simulate_with_correction(u, y0, p, correction_coefs, ts):
    u_delayed = np.concatenate([np.full(p["theta_steps"], u[0]), u])[: len(u)]
    y_ss = np.array([steady_state_from_params(p, uu, correction_coefs) for uu in u_delayed])
    sim = np.empty(len(u))
    sim[0] = y0
    alpha = 1.0 - np.exp(-ts / p["tau"])  # exact ZOH, matching simulator.py's _simulate_fopdt
    # y_ss[k+1], not y_ss[k] -- see simulator.py's _simulate_fopdt for why: sim[k+1]
    # must be driven by the same-time-index (delayed by theta) input as Simulator.step()
    # uses, not one sample earlier.
    for k in range(len(u) - 1):
        sim[k + 1] = sim[k] + alpha * (y_ss[k + 1] - sim[k])
    return sim


def fit_residual_correction(df, model, degree=1):
    """Hybrid physics+ML layer: fit a small (degree-1 by default -- "linear or
    shallow", per spec) polynomial-in-choke correction on the residual the physics
    FOPDT model leaves on a step test, per channel. This is a bias-correction on the
    steady-state map, not a replacement for the physics dynamics -- deliberately tiny
    (degree+1 coefficients) so it corrects systematic curve-shape mismatch without
    having enough freedom to just memorize the training step test.
    """
    u = df["Choke_pct"].to_numpy(dtype=float)
    correction = {}
    for ch, p in model.items():
        y = df[ch].to_numpy(dtype=float)
        physics_pred = _simulate_fopdt(u, y[0], p["A"], p["B"], p["uh"], p["tau"],
                                        p["theta_steps"], TS_HOURS, clip_nonneg=p["clip_nonneg"])
        resid = y - physics_pred
        correction[ch] = np.polyfit(u, resid, degree)
    return correction


def evaluate_correction(model, correction, holdout_df):
    """Physics-only vs physics+correction RMSE on a held-out step test (not the data
    the correction was fit on) -- the honest way to check the correction generalizes
    rather than just fitting noise in the training run."""
    u = holdout_df["Choke_pct"].to_numpy(dtype=float)
    report = {}
    for ch, p in model.items():
        y = holdout_df[ch].to_numpy(dtype=float)
        physics_only = _simulate_fopdt(u, y[0], p["A"], p["B"], p["uh"], p["tau"],
                                        p["theta_steps"], TS_HOURS, clip_nonneg=p["clip_nonneg"])
        corrected = _simulate_with_correction(u, y[0], p, correction[ch], TS_HOURS)
        rmse_before = float(np.sqrt(np.mean((physics_only - y) ** 2)))
        rmse_after = float(np.sqrt(np.mean((corrected - y) ** 2)))
        report[ch] = (rmse_before, rmse_after)
    return report


def select_beneficial_corrections(correction, report):
    """Only keep the learned correction for channels where it actually reduced
    held-out RMSE; channels where it didn't generalize fall back to physics-only
    (None). A correction that doesn't demonstrably help has no business being wired
    into the controller just because it exists."""
    return {ch: (coefs if report[ch][1] < report[ch][0] else None)
            for ch, coefs in correction.items()}


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

    correction = fit_residual_correction(df, model)
    holdout_df = run_step_test(seed=99)  # different noise draw, same design -- generalization check
    report = evaluate_correction(model, correction, holdout_df)
    print("physics-only vs physics+learned-correction RMSE on a held-out step test:")
    for ch, (before, after) in report.items():
        verdict = "helps" if after < before else "does NOT help"
        print(f"  {ch}: {before:.3f} -> {after:.3f}  ({verdict})")
    print()
    demo()
