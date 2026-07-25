"""
Scenario C deliberately requests an infeasible 400 bbl/hr target. The controller
must never let a candidate move breach a TRUE WHP/FLP/BHP limit -- constraint
tightening (controller.py's 3-sigma noise margin) only changes which candidate gets
picked, it never loosens the real-world limit itself.

Swept across the same 30 noise seeds seed_sweep.py uses: a single seed is a point
sample, not a safety claim (see docs/report.md Sec 3.4). Confirmed empirically clean
(0/30 seeds, 0 violations) after the DWELL_HOURS fix in identify.py improved the
identified model's noise_std/tau accuracy, which feeds directly into that tightening
margin -- re-check this test if identify.py's step-test design changes again.
"""
import pytest

from scenarios import run_scenario

N_SEEDS = 30


@pytest.mark.parametrize("seed", range(N_SEEDS))
def test_infeasible_target_never_breaches_true_limit(identified_system, seed):
    model, limits, correction = (identified_system[k] for k in ("model", "limits", "correction"))
    df = run_scenario("C_infeasible_target", identified_system["u_stable_100"],
                       (lambda t: 400.0), 100, model, limits, sim_seed=seed, correction=correction,
                       save=False)
    for ch in ("WHP", "FLP", "BHP"):
        lo, hi = limits[ch]
        assert (df[ch] >= lo).all() and (df[ch] <= hi).all(), (
            f"seed {seed}: {ch} breached the true limit ({lo}, {hi})"
        )
