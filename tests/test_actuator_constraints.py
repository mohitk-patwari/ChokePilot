"""
Actuator-level invariants that must hold no matter what the controller decides:
choke position and the +-5%/interval ramp-rate limit are hard constraints enforced
inside MPCController.decide()/commit() (controller.py), not just a design goal.
Checked here against the real scenario runner (scenarios.run_scenario), across all
three scenario shapes -- a startup ramp, a mid-run target step, and a controller
pinned against an infeasible target -- since each stresses the actuator logic
differently.
"""
import pandas as pd
import pytest

from controller import CHOKE_MIN, CHOKE_MAX, MAX_RAMP_PCT
from scenarios import run_scenario

# (name, initial_choke or None for "the model's ~100 bbl/hr stable point", target_fn,
# hours, sim_seed) -- mirrors scenarios.py's own scenario specs exactly.
SCENARIOS = [
    ("A_startup_to_target", 15.0, (lambda t: 100.0), 80, 10),
    ("B_target_tracking", None, (lambda t: 100.0 if t < 60 else 150.0), 140, 11),
    ("C_infeasible_target", None, (lambda t: 400.0), 100, 12),
]


@pytest.fixture(params=SCENARIOS, ids=[s[0] for s in SCENARIOS])
def scenario_run(request, identified_system):
    name, u0, target_fn, hours, seed = request.param
    if u0 is None:
        u0 = identified_system["u_stable_100"]
    df = run_scenario(name, u0, target_fn, hours, identified_system["model"],
                       identified_system["limits"], sim_seed=seed,
                       correction=identified_system["correction"])
    return u0, df


def test_choke_stays_within_bounds(scenario_run):
    _, df = scenario_run
    assert (df.Choke >= CHOKE_MIN).all()
    assert (df.Choke <= CHOKE_MAX).all()


def test_ramp_rate_never_exceeds_limit(scenario_run):
    initial_choke, df = scenario_run
    # Include the very first commanded move (from the scenario's starting choke),
    # not just moves between logged rows -- diff() on df.Choke alone would miss it.
    moves = pd.concat([pd.Series([initial_choke]), df.Choke], ignore_index=True).diff().dropna().abs()
    assert (moves <= MAX_RAMP_PCT + 1e-9).all()
