import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "data"))  # so `import simulator` works from any test file

import pytest  # noqa: E402

from identify import (  # noqa: E402
    run_step_test, identify_model, fit_residual_correction,
    evaluate_correction, select_beneficial_corrections,
)
from controller import safety_limits_from_reference  # noqa: E402
from scenarios import solve_choke_for_q, CSV_PATH  # noqa: E402


@pytest.fixture(scope="session")
def identified_system():
    """One identified model + true limits + learned correction, shared across the
    suite -- the exact same setup scenarios.py/seed_sweep.py run, so tests exercise
    the real pipeline rather than a synthetic stand-in."""
    step_df = run_step_test(seed=0)
    model = identify_model(step_df)
    limits = safety_limits_from_reference(CSV_PATH)
    raw_correction = fit_residual_correction(step_df, model)
    holdout_df = run_step_test(seed=99)
    report = evaluate_correction(model, raw_correction, holdout_df)
    correction = select_beneficial_corrections(raw_correction, report)
    return {
        "model": model,
        "limits": limits,
        "correction": correction,
        "u_stable_100": solve_choke_for_q(model, 100.0),
    }
