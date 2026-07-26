"""
Scenario D - Disturbance Rejection.

Deliberate, explicit relaxation of the challenge brief's "no changing reservoir
properties" simplification, done specifically to test whether re-planning from live
measurements (MPC) is actually more robust than a fixed, set-once setpoint
(baselines.py's Fixed) when the real plant drifts underneath both of them.

Setup: BHP's identified steady-state offset ("A" in y_ss = A + B*u/(u+uh)) drifts
-0.5 psi/h for 200h -- a slow reservoir pressure decline -- while the target oil
rate is held constant at 100 bbl/hr. The IDENTIFIED model both controllers plan
against is fit once, before the disturbance starts, and never updated, matching how
a real deployment re-identifies occasionally rather than continuously -- neither
approach "knows" the reservoir is declining; only MPC re-measures and re-plans.
"""

import copy
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import Simulator, PARAMS as SIM_PARAMS  # noqa: E402

from identify import (  # noqa: E402
    run_step_test, identify_model, fit_residual_correction,
    evaluate_correction, select_beneficial_corrections,
)
from controller import (  # noqa: E402
    MPCController, safety_limits_from_reference, CHOKE_MIN, CHOKE_MAX, MAX_RAMP_PCT,
)
from scenarios import solve_choke_for_q, check_constraints, violation_mask, CSV_PATH  # noqa: E402
from baselines import fixed_choke_setpoint, operator_proxy_setpoint  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)

HOURS = 200
BHP_DRIFT_PER_HOUR = -0.5  # psi/h -- see module docstring
TARGET_Q = 100.0
COLUMNS = ["Time_hr", "Target_Q", "Q", "WHP", "FLP", "BHP", "Choke", "Why"]


def run_mpc(model, limits, correction, start_choke, sim_seed):
    drift_params = copy.deepcopy(SIM_PARAMS)  # private plant copy -- see Simulator's params= doc
    sim = Simulator(initial_choke=start_choke, seed=sim_seed, params=drift_params)
    ctrl = MPCController(model, limits, correction=correction)
    choke = start_choke
    q, whp, flp, bhp = sim._read()
    rows = []
    for t in range(HOURS):
        drift_params["BHP"]["A"] += BHP_DRIFT_PER_HOUR
        state = {"Q": q, "WHP": whp, "FLP": flp, "BHP": bhp}
        choke, why = ctrl.decide(state, choke, TARGET_Q)
        ctrl.commit(choke)
        q, whp, flp, bhp = sim.step(choke)
        rows.append((t, TARGET_Q, q, whp, flp, bhp, choke, why))
    return pd.DataFrame(rows, columns=COLUMNS)


def run_fixed(setpoint, start_choke, sim_seed):
    """setpoint is computed once by the caller (fixed_choke_setpoint for Fixed-optimal,
    operator_proxy_setpoint for Fixed-operator-proxy) and held for the whole run --
    each independent copy of the drifting plant, so the two baselines and MPC all see
    the identical disturbance, not a shared/compounded one."""
    drift_params = copy.deepcopy(SIM_PARAMS)
    sim = Simulator(initial_choke=start_choke, seed=sim_seed, params=drift_params)
    choke = start_choke
    rows = []
    for t in range(HOURS):
        drift_params["BHP"]["A"] += BHP_DRIFT_PER_HOUR
        choke = float(np.clip(choke + np.clip(setpoint - choke, -MAX_RAMP_PCT, MAX_RAMP_PCT),
                               CHOKE_MIN, CHOKE_MAX))
        q, whp, flp, bhp = sim.step(choke)
        rows.append((t, TARGET_Q, q, whp, flp, bhp, choke, f"held at setpoint {setpoint:.1f}%"))
    return pd.DataFrame(rows, columns=COLUMNS)


def time_to_first_violation(df, limits):
    viol = violation_mask(df, limits)
    return float(df.loc[viol, "Time_hr"].iloc[0]) if viol.any() else None


_RUN_COLORS = {"MPC": "tab:blue", "Fixed-optimal": "tab:orange", "Fixed-operator-proxy": "tab:red"}


def plot_comparison(runs, limits, path):
    """runs: {label: df} -- one line per approach, same color scheme as baselines.py."""
    fig, axes = plt.subplots(5, 1, figsize=(11, 16), sharex=True)
    any_df = next(iter(runs.values()))

    axes[0].plot(any_df.Time_hr, any_df.Target_Q, "--", color="gray", label="Target")
    for label, df in runs.items():
        axes[0].plot(df.Time_hr, df.Q, color=_RUN_COLORS[label], label=label)
    axes[0].set_ylabel("Oil Rate (bbl/hr)")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].set_title("Scenario D - Disturbance Rejection (BHP declining -0.5 psi/h, 200h)")

    for ax, col, label in zip(axes[1:4], ["WHP", "FLP", "BHP"],
                               ["WHP (psi)", "FLP (psi)", "BHP (psi)"]):
        for run_label, df in runs.items():
            ax.plot(df.Time_hr, df[col], color=_RUN_COLORS[run_label], label=run_label)
        lo, hi = limits[col]  # one-sided limits use +-inf for "no bound that side"
        if math.isfinite(lo):
            ax.axhline(lo, color="red", linestyle=":", linewidth=1)
        if math.isfinite(hi):
            ax.axhline(hi, color="red", linestyle=":", linewidth=1)
        ax.set_ylabel(label)
    axes[1].legend(loc="best", fontsize=8)

    for label, df in runs.items():
        axes[4].plot(df.Time_hr, df.Choke, color=_RUN_COLORS[label], label=label)
    axes[4].set_ylabel("Choke (%)")
    axes[4].set_ylim(-2, 102)
    axes[4].set_xlabel("Time (hr)")
    axes[4].legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"saved {path}")


def main():
    step_df = run_step_test(seed=0)
    model = identify_model(step_df)
    limits = safety_limits_from_reference(CSV_PATH)
    raw_correction = fit_residual_correction(step_df, model)
    holdout_df = run_step_test(seed=99)
    correction = select_beneficial_corrections(raw_correction, evaluate_correction(model, raw_correction, holdout_df))
    tightened = MPCController(model, limits, correction=correction).limits
    start_choke = solve_choke_for_q(model, TARGET_Q)
    print(f"start choke (stable point for {TARGET_Q} bbl/hr): {start_choke:.1f}%\n")

    seed = 20  # same seed for every run -- identical noise draw, isolates the strategy
    optimal_setpoint = fixed_choke_setpoint(model, correction, tightened, TARGET_Q)
    operator_setpoint = operator_proxy_setpoint(CSV_PATH, TARGET_Q)
    print(f"Fixed-optimal setpoint: {optimal_setpoint:.1f}%   "
          f"Fixed-operator-proxy setpoint: {operator_setpoint:.1f}%\n")

    runs = {
        "MPC": run_mpc(model, limits, correction, start_choke, sim_seed=seed),
        "Fixed-optimal": run_fixed(optimal_setpoint, start_choke, sim_seed=seed),
        "Fixed-operator-proxy": run_fixed(operator_setpoint, start_choke, sim_seed=seed),
    }
    for label, df in runs.items():
        tag = label.lower().replace("-", "_")
        df.to_csv(OUTPUT_DIR / f"scenario_D_{tag}.csv", index=False)
    plot_comparison(runs, limits, OUTPUT_DIR / "scenario_D_disturbance_rejection.png")

    print(f"{'approach':<22} {'violations':>10} {'barrels':>10} {'time_to_first_violation':>24}")
    for label, df in runs.items():
        viol, _ = check_constraints(f"D_{label}", df, limits)
        barrels = float(df.Q.sum())
        t_first = time_to_first_violation(df, limits)
        t_str = f"{t_first:.0f}h" if t_first is not None else "never"
        print(f"{label:<22} {viol:>10d} {barrels:>10.1f} {t_str:>24}")


def demo():
    """Self-check: the per-instance drift must never leak into the shared, global
    PARAMS other scenarios rely on, and must accumulate exactly as commanded."""
    before = SIM_PARAMS["BHP"]["A"]
    drift_params = copy.deepcopy(SIM_PARAMS)
    sim = Simulator(initial_choke=30.0, seed=0, params=drift_params)
    for _ in range(10):
        drift_params["BHP"]["A"] += BHP_DRIFT_PER_HOUR
        sim.step(30.0)
    assert abs(drift_params["BHP"]["A"] - (before + 10 * BHP_DRIFT_PER_HOUR)) < 1e-9
    assert SIM_PARAMS["BHP"]["A"] == before, "drift leaked into the shared global PARAMS"
    print("scenario_d.py self-check PASSED (drift isolated, accumulates correctly)")


if __name__ == "__main__":
    demo()
    print()
    main()
