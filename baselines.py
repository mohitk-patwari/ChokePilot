"""
Baselines to compare against controller.py's brute-force MPC, run over the same
scenarios, same identified model, same safety limits, same simulator seeds as
scenarios.py -- so outputs/baseline_comparison.csv is an apples-to-apples comparison.

  1. Fixed-optimal -- computed once per target from the identified model's own Q(u)
     steady-state map, capped so its own steady-state pressure predictions stay inside
     the safety envelope, then held. Still model-informed and envelope-aware -- this
     is what a good engineer with the identification pipeline would set, not what a
     real operator without it would. Models the "set once, left alone" half of pain
     point #2 (CLAUDE.md), but NOT pain point #1's conservatism, since it still
     optimizes against the real envelope.
  2. Fixed-operator-proxy -- NO model, NO envelope knowledge at all: a naive linear
     read of choke-vs-oil-rate straight off the raw reference CSV (the only "real"
     data an operator without this pipeline would have), then backed off ~15
     percentage points from wherever that naive line says the target is met. Models
     pain point #1 directly: "operators baby the choke conservatively (fear of sand/
     formation damage), leaving real production capacity unused" -- this baseline
     always under-produces relative to target by construction, on purpose.
  3. PI on oil rate -- velocity-form PI (Delta-u = Kp*(e_k-e_{k-1}) + Ki*Ts*e_k), IMC-
     tuned from the identified Q model's own tau/theta/gain. The +-5%/interval ramp
     clamp on Delta-u doubles as its anti-windup -- there's no separate integral state
     to wind up beyond what's already expressed through the clamped Delta-u. Blind to
     WHP/FLP/BHP -- that blindness is the point of the comparison, since the MPC is
     safety-aware by construction and these baselines are not.

All three share controller.py's hard +-5%/interval actuator ramp limit; none replans
with a lookahead the way the MPC does.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import Simulator, TS_HOURS, steady_state_from_params  # noqa: E402

from identify import (  # noqa: E402
    run_step_test, identify_model, fit_residual_correction,
    evaluate_correction, select_beneficial_corrections,
)
from controller import (  # noqa: E402
    MPCController, safety_limits_from_reference, CHOKE_MIN, CHOKE_MAX, MAX_RAMP_PCT,
)
from scenarios import solve_choke_for_q, run_scenario, check_constraints  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)
CSV_PATH = Path(__file__).parent / "data" / "Autonomous_Choke_Control_Simulated_Dataset.csv"


def fixed_choke_setpoint(model, correction, tightened_limits, target_q):
    """The choke the identified model itself says holds target_q at steady state,
    walked back down (brute-force scan, 0.5%-steps) until its own steady-state
    pressure predictions clear the same tightened envelope the MPC uses. Models an
    operator's one-time, model-informed but conservative setpoint -- not a live
    feedback loop."""
    naive_u = solve_choke_for_q(model, target_q)
    for u in np.arange(naive_u, -0.5, -0.5):
        if all(tightened_limits[ch][0] <= steady_state_from_params(model[ch], u, correction.get(ch)) <= tightened_limits[ch][1]
               for ch in ("WHP", "FLP", "BHP")):
            return float(u)
    return 0.0


def operator_proxy_setpoint(csv_path, target_q, backoff_pct=15.0):
    """A real operator's mental model, not ours: a naive straight-line read of
    choke-vs-oil-rate off the raw reference CSV (no FOPDT fit, no steady-state curve,
    no pressure channels looked at, no envelope knowledge whatsoever -- literally the
    plot a field engineer might eyeball), then a conservative backoff. Models pain
    point #1 directly: operators baby the choke, leaving real production capacity on
    the table. Uninformed and un-optimized on purpose -- do not sharpen this."""
    df = pd.read_csv(csv_path)
    slope, intercept = np.polyfit(df["Choke_pct"], df["OilRate_bbl_hr"], 1)
    u_believed = (target_q - intercept) / slope
    return float(np.clip(u_believed - backoff_pct, CHOKE_MIN, CHOKE_MAX))


def run_fixed_baseline(name, initial_choke, target_fn, hours, setpoint_fn, sim_seed, tag=""):
    """setpoint_fn(target) computes the (held, ramp-limited) choke for a given
    target -- the only thing that differs between Fixed-optimal (fixed_choke_setpoint)
    and Fixed-operator-proxy (operator_proxy_setpoint) is which function is passed."""
    sim = Simulator(initial_choke=initial_choke, seed=sim_seed)
    choke = initial_choke
    q, whp, flp, bhp = sim._read()
    setpoint, prev_target = None, object()
    rows = []
    for t in range(hours):
        target = target_fn(t)
        if target != prev_target:
            setpoint = setpoint_fn(target)
            prev_target = target
        choke = float(np.clip(choke + np.clip(setpoint - choke, -MAX_RAMP_PCT, MAX_RAMP_PCT), CHOKE_MIN, CHOKE_MAX))
        q, whp, flp, bhp = sim.step(choke)
        rows.append((t, target, q, whp, flp, bhp, choke))
    df = pd.DataFrame(rows, columns=["Time_hr", "Target_Q", "Q", "WHP", "FLP", "BHP", "Choke"])
    df.to_csv(OUTPUT_DIR / f"baseline_fixed{tag}_{name}.csv", index=False)
    return df


def pi_gains(model, u_ref):
    """IMC tuning for a velocity-form PI from the identified Q model's own tau/theta
    and its local process gain dQss/du at u_ref -- no hand-picked magic numbers."""
    p = model["Q"]
    tau, theta = p["tau"], p["theta_steps"] * TS_HOURS
    gain = p["B"] * p["uh"] / (u_ref + p["uh"]) ** 2
    lam = max(theta, 0.3 * tau)  # IMC rule of thumb: closed-loop speed no faster than dead time
    Kp = tau / (gain * (lam + theta))
    Ki = Kp / tau  # Ti = tau
    return Kp, Ki


def run_pi_baseline(name, initial_choke, target_fn, hours, sim_seed, Kp, Ki):
    sim = Simulator(initial_choke=initial_choke, seed=sim_seed)
    choke = initial_choke
    q, whp, flp, bhp = sim._read()
    prev_err = target_fn(0) - q
    rows = []
    for t in range(hours):
        target = target_fn(t)
        err = target - q
        d_u = float(np.clip(Kp * (err - prev_err) + Ki * TS_HOURS * err, -MAX_RAMP_PCT, MAX_RAMP_PCT))
        choke = float(np.clip(choke + d_u, CHOKE_MIN, CHOKE_MAX))
        prev_err = err
        q, whp, flp, bhp = sim.step(choke)
        rows.append((t, target, q, whp, flp, bhp, choke))
    df = pd.DataFrame(rows, columns=["Time_hr", "Target_Q", "Q", "WHP", "FLP", "BHP", "Choke"])
    df.to_csv(OUTPUT_DIR / f"baseline_pi_{name}.csv", index=False)
    return df


def step_response_metrics(df, step_time, before_target, after_target):
    """Standard step-response metrics for a target step: 10-90% rise time, +-5%-of-
    step-size settling time, overshoot (absolute and % of step), and steady-state
    offset (mean of the last 10h vs. the new target)."""
    post = df[df.Time_hr >= step_time].reset_index(drop=True)
    t = post.Time_hr.to_numpy() - step_time
    q = post.Q.to_numpy()
    delta = after_target - before_target

    def first_cross(level):
        hit = np.where(q >= level)[0] if delta > 0 else np.where(q <= level)[0]
        return t[hit[0]] if len(hit) else np.nan

    t10 = first_cross(before_target + 0.1 * delta)
    t90 = first_cross(before_target + 0.9 * delta)
    rise_time = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan

    tol = 0.05 * abs(delta)
    within = np.abs(q - after_target) <= tol
    settle_idx = next((i for i in range(len(within)) if within[i:].all()), None)
    settling_time = t[settle_idx] if settle_idx is not None else np.nan

    overshoot = max(0.0, q.max() - after_target) if delta > 0 else max(0.0, after_target - q.min())
    tail = q[-10:] if len(q) >= 10 else q

    return {
        "rise_time_hr": rise_time,
        "settling_time_hr": settling_time,
        "overshoot_bbl_hr": overshoot,
        "overshoot_pct_of_step": 100.0 * overshoot / abs(delta),
        "steady_state_offset_bbl_hr": float(np.mean(tail) - after_target),
    }


def summarize(name, approach, df, limits, step_metrics=None):
    violations, ramp_violations = check_constraints(f"{approach}/{name}", df, limits)
    row = {
        "scenario": name,
        "approach": approach,
        "hours": len(df),
        "total_barrels": float(df.Q.sum()),  # Ts = 1h, so sum(Q [bbl/hr]) = bbl produced
        "safety_violations": violations,
        "ramp_violations": ramp_violations,
    }
    row.update(step_metrics or {})
    return row


_APPROACH_COLORS = {
    "MPC": "tab:blue", "Fixed-optimal": "tab:orange",
    "Fixed-operator-proxy": "tab:red", "PI": "tab:green",
}
_SCENARIO_LABELS = {"A_startup_to_target": "A", "B_target_tracking": "B", "C_infeasible_target": "C"}


def plot_comparison(df):
    """Grouped bar chart across all approaches: safety violations (log-scale -- PI's
    Scenario C count dwarfs everything else) and total barrels produced, per scenario.
    The single-figure story this project's results boil down to."""
    scenarios = list(_SCENARIO_LABELS)
    approaches = [a for a in _APPROACH_COLORS if a in set(df.approach)]
    x = np.arange(len(scenarios))
    width = 0.8 / len(approaches)

    fig, (ax_viol, ax_bbl) = plt.subplots(1, 2, figsize=(12, 4.5))
    for i, approach in enumerate(approaches):
        sub = df[df.approach == approach].set_index("scenario").loc[scenarios]
        offset = (i - (len(approaches) - 1) / 2) * width
        color = _APPROACH_COLORS[approach]
        ax_viol.bar(x + offset, sub.safety_violations.clip(lower=0.1), width, color=color, label=approach)
        ax_bbl.bar(x + offset, sub.total_barrels, width, color=color, label=approach)

    ax_viol.set_yscale("log")
    ax_viol.set_ylabel("Safety violations (log scale)")
    ax_viol.set_title("Constraint violations by approach")
    ax_bbl.set_ylabel("Total barrels produced")
    ax_bbl.set_title("Production by approach")
    for ax in (ax_viol, ax_bbl):
        ax.set_xticks(x)
        ax.set_xticklabels([_SCENARIO_LABELS[s] for s in scenarios])
        ax.set_xlabel("Scenario")
        ax.legend(fontsize=8)

    fig.suptitle("Baseline comparison: MPC vs. Fixed-optimal vs. Fixed-operator-proxy vs. PI")
    fig.tight_layout()
    path = OUTPUT_DIR / "baseline_comparison.png"
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

    # Reuse the MPC's own tightening (constraint-tightening margin, see controller.py)
    # so the fixed baseline is held to the same noise-robust envelope, not a looser one.
    tightened = MPCController(model, limits, correction=correction).limits
    u_stable_100 = solve_choke_for_q(model, 100.0)
    Kp, Ki = pi_gains(model, u_stable_100)
    print(f"PI gains (IMC, linearized at {u_stable_100:.1f}% choke): Kp={Kp:.4f} Ki={Ki:.4f}\n")

    # Same starts/targets/hours/seeds as scenarios.py, so all three approaches see
    # identical noise realizations per scenario.
    specs = [
        ("A_startup_to_target", 15.0, (lambda t: 100.0), 80, 10),
        ("B_target_tracking", u_stable_100, (lambda t: 100.0 if t < 60 else 150.0), 140, 11),
        ("C_infeasible_target", u_stable_100, (lambda t: 400.0), 100, 12),
    ]

    rows = []
    for name, u0, target_fn, hours, seed in specs:
        step_kwargs = dict(step_time=60, before_target=100.0, after_target=150.0) if name.startswith("B_") else None

        df_mpc = run_scenario(name, u0, target_fn, hours, model, limits, sim_seed=seed,
                               correction=correction, save=False)
        df_fixed = run_fixed_baseline(
            name, u0, target_fn, hours,
            setpoint_fn=lambda target: fixed_choke_setpoint(model, correction, tightened, target),
            sim_seed=seed)
        df_operator = run_fixed_baseline(
            name, u0, target_fn, hours,
            setpoint_fn=lambda target: operator_proxy_setpoint(CSV_PATH, target),
            sim_seed=seed, tag="_operator")
        df_pi = run_pi_baseline(name, u0, target_fn, hours, sim_seed=seed, Kp=Kp, Ki=Ki)

        for approach, df in [("MPC", df_mpc), ("Fixed-optimal", df_fixed),
                             ("Fixed-operator-proxy", df_operator), ("PI", df_pi)]:
            metrics = step_response_metrics(df, **step_kwargs) if step_kwargs else None
            rows.append(summarize(name, approach, df, limits, metrics))
        print()

    out = pd.DataFrame(rows)
    out.to_csv(OUTPUT_DIR / "baseline_comparison.csv", index=False)
    print(out.to_string(index=False))
    print(f"\nsaved {OUTPUT_DIR / 'baseline_comparison.csv'}")
    plot_comparison(out)


if __name__ == "__main__":
    main()
