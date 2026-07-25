"""
Runs Scenarios A/B/C against data/simulator.py using controller.py's MPCController,
built on the model identify.py identifies from a step test. Saves the required plot
set (target vs actual oil rate, WHP, FLP, BHP, choke position) per scenario.

Scenario A - Startup to Target: well starts shut in (choke 0%), controller brings it
to a 100 bbl/hr target.
Scenario B - Target Tracking: target steps 100 -> 150 bbl/hr partway through.
Scenario C - Infeasible Target: a 400 bbl/hr target exceeds what's achievable within
the safety limits; the controller should settle at the maximum safe rate instead.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import Simulator, TS_HOURS  # noqa: E402

from identify import run_step_test, identify_model  # noqa: E402
from controller import MPCController, safety_limits_from_reference  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)
CSV_PATH = Path(__file__).parent / "data" / "Autonomous_Choke_Control_Simulated_Dataset.csv"


def run_scenario(name, initial_choke, target_fn, hours, model, limits, sim_seed):
    sim = Simulator(initial_choke=initial_choke, seed=sim_seed)
    ctrl = MPCController(model, limits)
    choke = initial_choke
    q, whp, flp, bhp = sim._read()

    rows = []
    for t in range(hours):
        target = target_fn(t)
        state = {"Q": q, "WHP": whp, "FLP": flp, "BHP": bhp}
        choke, explanation = ctrl.decide(state, choke, target)
        q, whp, flp, bhp = sim.step(choke)
        rows.append((t, target, q, whp, flp, bhp, choke, explanation))

    df = pd.DataFrame(rows, columns=["Time_hr", "Target_Q", "Q", "WHP", "FLP", "BHP", "Choke", "Why"])
    df.to_csv(OUTPUT_DIR / f"scenario_{name}.csv", index=False)
    return df


def plot_scenario(name, title, df, limits):
    fig, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=True)

    axes[0].plot(df.Time_hr, df.Target_Q, "--", color="gray", label="Target")
    axes[0].plot(df.Time_hr, df.Q, color="tab:blue", label="Actual")
    axes[0].set_ylabel("Oil Rate (bbl/hr)")
    axes[0].legend(loc="lower right")
    axes[0].set_title(title)

    for ax, col, label, key in zip(axes[1:4], ["WHP", "FLP", "BHP"],
                                    ["WHP (psi)", "FLP (psi)", "BHP (psi)"],
                                    ["WHP", "FLP", "BHP"]):
        ax.plot(df.Time_hr, df[col], color="tab:blue")
        lo, hi = limits[key]
        ax.axhline(lo, color="red", linestyle=":", linewidth=1)
        ax.axhline(hi, color="red", linestyle=":", linewidth=1)
        ax.set_ylabel(label)

    axes[4].step(df.Time_hr, df.Choke, where="post", color="black")
    axes[4].set_ylabel("Choke (%)")
    axes[4].set_ylim(-2, 102)
    axes[4].set_xlabel("Time (hr)")

    fig.tight_layout()
    path = OUTPUT_DIR / f"scenario_{name}.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"saved {path}")


def check_constraints(name, df, limits):
    violations = 0
    for ch in ("WHP", "FLP", "BHP"):
        lo, hi = limits[ch]
        violations += int(((df[ch] < lo) | (df[ch] > hi)).sum())
    ramp_violations = int((df.Choke.diff().abs() > 5.0 + 1e-6).sum())
    print(f"[{name}] constraint samples outside limits: {violations}/{len(df)}   "
          f"ramp-rate violations: {ramp_violations}/{len(df)}")
    return violations, ramp_violations


def main():
    step_df = run_step_test(seed=0)
    model = identify_model(step_df)
    limits = safety_limits_from_reference(CSV_PATH)
    print("safety limits (placeholder, derived from reference data + 20% margin):", limits)
    print(f"model tau (h): {{ {', '.join(f'{ch}: {p['tau']:.2f}' for ch, p in model.items())} }}")

    scenarios = [
        ("A_startup_to_target", 0.0, (lambda t: 100.0), 80, 10),
        ("B_target_tracking", 0.0, (lambda t: 100.0 if t < 60 else 150.0), 140, 11),
        ("C_infeasible_target", 0.0, (lambda t: 400.0), 100, 12),
    ]
    titles = {
        "A_startup_to_target": "Scenario A - Startup to Target (0% choke -> 100 bbl/hr)",
        "B_target_tracking": "Scenario B - Target Tracking (100 -> 150 bbl/hr at t=60h)",
        "C_infeasible_target": "Scenario C - Infeasible Target (400 bbl/hr requested)",
    }

    for name, u0, target_fn, hours, seed in scenarios:
        df = run_scenario(name, u0, target_fn, hours, model, limits, sim_seed=seed)
        plot_scenario(name, titles[name], df, limits)
        check_constraints(name, df, limits)
        print(f"[{name}] final: target={df.Target_Q.iloc[-1]:.1f} actual={df.Q.iloc[-1]:.1f} "
              f"bbl/hr, choke={df.Choke.iloc[-1]:.0f}%")
        print(f"[{name}] sample decision: {df.Why.iloc[-1]}")
        print()


if __name__ == "__main__":
    main()
