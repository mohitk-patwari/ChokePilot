"""
Diagnostic only -- fixes nothing.

Compares identify.py's identified tau/theta per channel (fit from a fresh step
test run against the calibrated simulator) against the TRUE tau/theta hardcoded
in simulator.PARAMS (what the simulator actually uses internally, fit from the
reference CSV). If identification is working correctly, a fresh step test against
the simulator should recover values close to the simulator's own ground truth --
any large, consistent gap means something in identify.py's step-test design or
fitting procedure is biased, not just noisy.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import PARAMS as TRUE_PARAMS  # noqa: E402

from identify import run_step_test, identify_model, CHANNELS, DWELL_HOURS  # noqa: E402
from results_io import update_results  # noqa: E402

SEEDS = [0, 1, 2, 7, 99]


def main():
    rows = []
    for seed in SEEDS:
        df = run_step_test(seed=seed)
        model = identify_model(df)
        # Only the 4 channels identify.py actually fits from data -- simulator.PARAMS
        # also holds WHT/AP now, which are hand-set placeholders (see data/simulator.py),
        # never identified from a step test, so they have no "true" value to compare.
        for ch in CHANNELS:
            true_p = TRUE_PARAMS[ch]
            ident_p = model[ch]
            tau_err_pct = 100.0 * abs(ident_p["tau"] - true_p["tau"]) / true_p["tau"]
            rows.append({
                "seed": seed, "channel": ch,
                "true_tau": true_p["tau"], "ident_tau": ident_p["tau"], "tau_err_pct": tau_err_pct,
                "true_theta": true_p["theta_steps"], "ident_theta": ident_p["theta_steps"],
            })

    header = f"{'seed':>4} {'ch':>4} {'true_tau':>9} {'ident_tau':>10} {'%err':>7} {'true_th':>8} {'ident_th':>9}"
    print(header)
    print("-" * len(header))
    for r in rows:
        print(f"{r['seed']:>4} {r['channel']:>4} {r['true_tau']:>9.2f} {r['ident_tau']:>10.2f} "
              f"{r['tau_err_pct']:>6.1f}% {r['true_theta']:>8d} {r['ident_theta']:>9d}")

    print()
    print("mean tau %% error by channel across seeds:")
    identification = {"dwell_hours": DWELL_HOURS, "seeds": SEEDS, "channels": {}}
    for ch in CHANNELS:
        errs = [r["tau_err_pct"] for r in rows if r["channel"] == ch]
        ident_taus = [r["ident_tau"] for r in rows if r["channel"] == ch]
        theta_matches = [r["ident_theta"] == r["true_theta"] for r in rows if r["channel"] == ch]
        true_p = TRUE_PARAMS[ch]
        print(f"  {ch}: {sum(errs) / len(errs):.1f}%  (min {min(errs):.1f}%, max {max(errs):.1f}%)")
        identification["channels"][ch] = {
            "true_tau": true_p["tau"], "true_theta": true_p["theta_steps"],
            "ident_tau_mean": sum(ident_taus) / len(ident_taus),
            "tau_error_pct_mean": sum(errs) / len(errs),
            "tau_error_pct_min": min(errs), "tau_error_pct_max": max(errs),
            "theta_match_rate": sum(theta_matches) / len(theta_matches),
        }
    update_results("identification", identification)


if __name__ == "__main__":
    main()
