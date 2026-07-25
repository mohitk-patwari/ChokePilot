"""
Brute-force predictive (MPC-style) choke controller.

Each control interval: evaluate a small, fixed set of candidate choke moves
(-5%..+5% in 1% steps -- the full range the ramp-rate constraint allows), simulate
each candidate forward over a lookahead horizon using the identified FOPDT model
(identify.py's identify_model output), reject any candidate predicted to violate a
WHP/FLP/BHP safety limit at any point in the horizon, and command the feasible
candidate whose end-of-horizon predicted oil rate is closest to target. If nothing is
feasible (e.g. noise has already pushed a reading near a limit), fall back to the
candidate with the smallest predicted constraint violation. Brute-force over 11 fixed
candidates is cheap enough that no optimizer is needed -- the challenge brief calls
this acceptable.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent / "data"))
from simulator import steady_state_from_params  # noqa: E402

CHOKE_MIN, CHOKE_MAX = 0.0, 100.0
MAX_RAMP_PCT = 5.0  # hard constraint: max |choke change| per control interval
CANDIDATE_DELTAS = np.arange(-MAX_RAMP_PCT, MAX_RAMP_PCT + 1, 1.0)  # 11 candidates, 1% resolution


def safety_limits_from_reference(csv_path, margin_frac=0.20, round_to=5.0):
    """Derive WHP/FLP/BHP safe operating limits from the reference dataset's observed
    range plus headroom, since the challenge brief doesn't specify numeric limits.

    PLACEHOLDER -- DERIVED FROM SAMPLE DATA, NOT OFFICIAL LIMITS. Replace with the
    real operating envelope if/when one is specified.

    Each channel is bounded on only the side that's actually a safety risk here --
    a symmetric two-sided band would reject a high-WHP/high-BHP or low-FLP state that
    is not actually unsafe, needlessly restricting achievable production:
    """
    df = pd.read_csv(csv_path)
    lo_hi = {}
    for ch, col in [("WHP", "WHP_psi"), ("FLP", "FLP_psi"), ("BHP", "BHP_psi")]:
        lo, hi = df[col].min(), df[col].max()
        margin = margin_frac * (hi - lo)
        lo_hi[ch] = (round_to * math.floor((lo - margin) / round_to),
                     round_to * math.ceil((hi + margin) / round_to))

    limits = {
        # Brief: "If WHP becomes too low, the well may operate outside its
        # recommended operating envelope." High WHP just means the choke is closed
        # back further (safe, conservative) -- only the floor is an active risk.
        "WHP": (lo_hi["WHP"][0], math.inf),
        # Brief: BHP is "one of the most important indicators of reservoir health
        # and drawdown" -- low BHP means excessive drawdown (sand/formation-damage
        # risk); high BHP means low drawdown, i.e. safely choked back. Floor only.
        "BHP": (lo_hi["BHP"][0], math.inf),
        # Brief: FLP "helps ensure stable transportation of produced fluids" --
        # the risk here is excess backpressure/separator overpressure on the high
        # side; a low FLP isn't a hazard this challenge's envelope cares about.
        "FLP": (-math.inf, lo_hi["FLP"][1]),
    }
    return limits


def _predict_horizon(model, state, u_history, new_u, horizon, ts, correction=None):
    """Simulate `horizon` steps ahead: apply new_u this interval, then hold it there.
    Returns {channel: array of length horizon} of predicted values."""
    correction = correction or {}
    u_seq = u_history + [new_u] * horizon
    base_len = len(u_history)
    preds = {ch: np.empty(horizon) for ch in model}
    cur = dict(state)
    for k in range(horizon):
        for ch, p in model.items():
            idx = max(0, base_len + k - p["theta_steps"])
            y_ss = steady_state_from_params(p, u_seq[idx], correction.get(ch))
            alpha = ts / p["tau"]
            cur[ch] = cur[ch] + alpha * (y_ss - cur[ch])
            preds[ch][k] = cur[ch]
    return preds


class MPCController:
    """One-sentence-explainable brute-force MPC: 11 fixed candidate moves, simulated
    over a lookahead horizon, filtered by safety, chosen by closeness to target."""

    def __init__(self, model, limits, ts_hours=1.0, horizon=None, noise_margin_sigma=3.0,
                 correction=None):
        self.model = model
        # Learned residual correction per channel (identify.py's fit_residual_correction,
        # filtered by select_beneficial_corrections) -- None per-channel where it didn't
        # demonstrably help on held-out data. Optional: physics-only if not supplied.
        self.correction = correction or {}
        self.true_limits = limits  # {"WHP": (lo, hi), "FLP": (lo, hi), "BHP": (lo, hi)}
        # Predictions are noise-free, but real readings carry measurement noise, so
        # riding the true limit exactly would let real noise dip past it. Back off
        # each limit by a few sigma of that channel's identified noise_std -- a
        # standard robust-MPC "constraint tightening" margin -- so the controller
        # targets a slightly tighter envelope and the true limit stays respected.
        self.limits = {
            ch: (lo + noise_margin_sigma * model[ch]["noise_std"],
                 hi - noise_margin_sigma * model[ch]["noise_std"])
            for ch, (lo, hi) in limits.items()
        }
        self.ts = ts_hours
        if horizon is None:
            max_tau = max(p["tau"] for p in model.values())
            horizon = int(np.clip(math.ceil(3 * max_tau / ts_hours), 3, 12))
        self.horizon = horizon
        self.u_history = []  # actual applied choke positions, for dead-time lookback

    def decide(self, state, current_choke, target_q):
        """Pure -- does not mutate controller state (safe to call speculatively,
        e.g. for a what-if check, without corrupting anything). state: dict with
        current Q, WHP, FLP, BHP readings. Returns (new_choke_position,
        one_sentence_explanation). Call commit(new_choke_position) once you've
        actually applied the move, so the next decide() call's dead-time lookback
        reflects what really happened -- calling decide() again without committing
        (or committing twice) previously corrupted u_history; now it's on the
        caller only if they skip commit() entirely.
        """
        # u_history should already end in current_choke (the caller committed the
        # last move), but on the very first call ever nothing has been committed --
        # fall back to a local seed for this call only, without writing to self.
        u_history = self.u_history if self.u_history else [current_choke]

        results = []
        for delta in CANDIDATE_DELTAS:
            # Round to 0.1% here, before it's used for prediction/selection/logging,
            # so the value that gets predicted against, applied, and quoted in the
            # rationale string below are all the exact same number -- not a full-
            # precision float that then gets displayed rounded to a different value.
            new_u = round(float(np.clip(current_choke + delta, CHOKE_MIN, CHOKE_MAX)), 1)
            preds = _predict_horizon(self.model, state, u_history, new_u, self.horizon,
                                      self.ts, self.correction)
            violation = 0.0
            for ch in ("WHP", "FLP", "BHP"):
                lo, hi = self.limits[ch]
                violation += float(np.sum(np.maximum(0.0, lo - preds[ch])))
                violation += float(np.sum(np.maximum(0.0, preds[ch] - hi)))
            results.append({
                "new_u": new_u,
                "violation": violation,
                "q_end": float(preds["Q"][-1]),
                "q_err": abs(float(preds["Q"][-1]) - target_q),
            })

        feasible = [r for r in results if r["violation"] == 0.0]
        if feasible:
            chosen = min(feasible, key=lambda r: (r["q_err"], abs(r["new_u"] - current_choke)))
            explanation = (
                f"Moved choke to {chosen['new_u']:.1f}% because it keeps WHP/FLP/BHP within "
                f"safe limits over the next {self.horizon}h and brings predicted oil rate to "
                f"{chosen['q_end']:.1f} bbl/hr, closest to the {target_q:.1f} bbl/hr target "
                f"among {len(feasible)} feasible options."
            )
        else:
            chosen = min(results, key=lambda r: (r["violation"], abs(r["new_u"] - current_choke)))
            explanation = (
                f"No choke move keeps all limits satisfied over the lookahead; moved to "
                f"{chosen['new_u']:.1f}% because it minimizes the predicted constraint "
                f"violation ({chosen['violation']:.1f} psi-steps over horizon)."
            )

        return chosen["new_u"], explanation

    def commit(self, applied_choke):
        """Record a choke move that was actually applied, into the dead-time
        lookback history. Call this exactly once per control interval, after
        applying decide()'s chosen move."""
        self.u_history.append(round(float(applied_choke), 1))


def demo():
    """Self-check using a real identified model: the controller should (a) open the
    choke when Q is below an achievable target and pressures have headroom, and
    (b) never command a choke position outside [0,100] or a move outside +-5%, even
    when every candidate is predicted to violate an impossibly tight limit."""
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent))
    from identify import run_step_test, identify_model

    df = run_step_test(seed=2)
    model = identify_model(df)
    csv_path = Path(__file__).parent / "data" / "Autonomous_Choke_Control_Simulated_Dataset.csv"
    limits = safety_limits_from_reference(csv_path)

    ctrl = MPCController(model, limits)
    state = {"Q": 90.0, "WHP": 260.0, "FLP": 185.0, "BHP": 3050.0}

    # decide() must be pure: calling it twice with identical inputs, without an
    # intervening commit(), has to return the identical result -- this is exactly
    # the case that used to corrupt u_history (each call appended a second, bogus
    # entry) and would have made these two calls disagree.
    new_u_a, why_a = ctrl.decide(state, current_choke=30.0, target_q=150.0)
    new_u_b, why_b = ctrl.decide(state, current_choke=30.0, target_q=150.0)
    assert new_u_a == new_u_b and why_a == why_b, "decide() is not pure -- repeat call diverged"

    new_u, why = new_u_a, why_a
    ctrl.commit(new_u)
    assert new_u > 30.0, "expected controller to open the choke toward an achievable higher target"
    assert new_u - 30.0 <= MAX_RAMP_PCT + 1e-9, "ramp-rate constraint violated"
    print(f"achievable-target case: {why}")

    # An obviously-unreachable tight finite band for every channel, independent of
    # whatever safety_limits_from_reference() produces -- WHP/BHP are lower-bounded
    # only (hi=inf) and FLP is upper-bounded only (lo=-inf) now, so naively
    # tightening around those real limits' finite side can hit +-inf and degenerate
    # the comparison instead of exercising it.
    impossible_limits = {ch: (1.0e6, 1.0e6 + 0.01) for ch in limits}
    ctrl2 = MPCController(model, impossible_limits)
    new_u2, why2 = ctrl2.decide(state, current_choke=30.0, target_q=150.0)
    ctrl2.commit(new_u2)
    assert 0.0 <= new_u2 <= 100.0
    assert abs(new_u2 - 30.0) <= MAX_RAMP_PCT + 1e-9
    print(f"impossible-limits case: {why2}")
    print("controller.py self-check PASSED")


if __name__ == "__main__":
    demo()
