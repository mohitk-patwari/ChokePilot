"""
Seed sweep: run each of Scenarios A/B/C across many simulator-noise seeds, with the
identified model/limits/correction held fixed (identified once, same as scenarios.py),
to check whether Scenario A's known residual violations (and how often the controller
falls back to the "no fully feasible candidate" branch) are a stable, seed-independent
finding or an artifact of the one noise draw scenarios.py happens to use by default.

Does not commit or touch scenarios.py's own default-seed outputs -- writes only to
outputs/seed_sweep_results.csv.
"""

import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).parent / "data"))

from identify import (  # noqa: E402
    run_step_test, identify_model, fit_residual_correction,
    evaluate_correction, select_beneficial_corrections,
)
from controller import safety_limits_from_reference  # noqa: E402
from scenarios import run_scenario, solve_choke_for_q, CSV_PATH  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)
N_SEEDS = 30


def count_violations(df, limits):
    viol = pd.Series(False, index=df.index)
    for ch in ("WHP", "FLP", "BHP"):
        lo, hi = limits[ch]
        viol |= (df[ch] < lo) | (df[ch] > hi)
    return int(viol.sum())


def fallback_fraction(df):
    """Fraction of steps where decide() hit the safety-fallback branch (no candidate
    fully clears the tightened envelope) -- both branches' text is defined in
    controller.py's decide(); the fallback one starts with "No choke move"."""
    return float(df.Why.str.startswith("No choke move").mean())


def main():
    step_df = run_step_test(seed=0)
    model = identify_model(step_df)
    limits = safety_limits_from_reference(CSV_PATH)
    raw_correction = fit_residual_correction(step_df, model)
    holdout_df = run_step_test(seed=99)
    correction_report = evaluate_correction(model, raw_correction, holdout_df)
    correction = select_beneficial_corrections(raw_correction, correction_report)
    u_stable_100 = solve_choke_for_q(model, 100.0)

    scenario_defs = [
        ("A_startup_to_target", 15.0, (lambda t: 100.0), 80),
        ("B_target_tracking", u_stable_100, (lambda t: 100.0 if t < 60 else 150.0), 140),
        ("C_infeasible_target", u_stable_100, (lambda t: 400.0), 100),
    ]

    rows = []
    for name, u0, target_fn, hours in scenario_defs:
        for seed in range(N_SEEDS):
            df = run_scenario(name, u0, target_fn, hours, model, limits,
                               sim_seed=seed, correction=correction)
            rows.append({
                "scenario": name,
                "seed": seed,
                "steps": len(df),
                "violations": count_violations(df, limits),
                "fallback_frac": fallback_fraction(df),
            })

    results = pd.DataFrame(rows)
    results.to_csv(OUTPUT_DIR / "seed_sweep_results.csv", index=False)
    print(f"saved {OUTPUT_DIR / 'seed_sweep_results.csv'}  ({len(results)} runs, "
          f"{N_SEEDS} seeds x {len(scenario_defs)} scenarios)\n")

    header = f"{'scenario':<22} {'mean_viol':>9} {'max_viol':>8} {'seeds>=1viol':>13} {'mean_fallback%':>15}"
    print(header)
    print("-" * len(header))
    for name, _, _, _ in scenario_defs:
        sub = results[results.scenario == name]
        n_with_viol = int((sub.violations >= 1).sum())
        print(f"{name:<22} {sub.violations.mean():>9.2f} {sub.violations.max():>8d} "
              f"{n_with_viol:>10d}/{len(sub):<3d}{100 * sub.fallback_frac.mean():>14.2f}%")


if __name__ == "__main__":
    main()
