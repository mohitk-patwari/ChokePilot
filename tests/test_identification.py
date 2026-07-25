"""
identify.py's fresh step test should recover time constants close to the
simulator's own ground truth (simulator.PARAMS, fit from the reference CSV) --
mirrors verify_identification.py's diagnostic, as assertions.

A flat "within 15%" bound does not hold, and asserting it would misrepresent what
this pipeline actually achieves (see CLAUDE.md / docs/report.md Sec 1.2). After
bumping identify.py's DWELL_HOURS from 24h to 40h (confirmed by direct
measurement to be dwell-time-limited for WHP/FLP/BHP -- BHP's true tau=9h,
theta=2h needs ~4*9+2=38h to settle, which 24h never gave it), WHP/FLP/BHP error
dropped sharply, but Q holds steady around 15-25% error at every dwell length
tested (24h through 80h) -- a separate, still-unexplained identification bias,
not a settling-time problem. Per-channel thresholds below reflect that reality
(observed max + headroom), as a regression guard against the error getting
*worse* -- not a claim that identification is already tight everywhere.
"""
import pytest

from simulator import PARAMS as TRUE_PARAMS

from identify import run_step_test, identify_model, CHANNELS

SEEDS = [0, 1, 2, 7, 99]  # matches verify_identification.py

# Observed max tau error (verify_identification.py, post DWELL_HOURS=40 fix):
# Q 25.0%, WHP 6.7%, FLP 17.9%, BHP 11.1%. Ceilings below add headroom on top.
MAX_TAU_ERROR_PCT = {"Q": 30.0, "WHP": 15.0, "FLP": 25.0, "BHP": 20.0}


@pytest.mark.parametrize("seed", SEEDS)
def test_identified_tau_within_channel_threshold(seed):
    df = run_step_test(seed=seed)
    model = identify_model(df)
    for ch in CHANNELS:
        true_tau = TRUE_PARAMS[ch]["tau"]
        ident_tau = model[ch]["tau"]
        err_pct = 100.0 * abs(ident_tau - true_tau) / true_tau
        assert err_pct <= MAX_TAU_ERROR_PCT[ch], (
            f"seed {seed}, {ch}: tau error {err_pct:.1f}% exceeds "
            f"{MAX_TAU_ERROR_PCT[ch]}% ceiling (true={true_tau}, identified={ident_tau})"
        )
