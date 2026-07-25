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

import numpy as np
import pandas as pd

CHOKE_MIN, CHOKE_MAX = 0.0, 100.0
MAX_RAMP_PCT = 5.0  # hard constraint: max |choke change| per control interval
CANDIDATE_DELTAS = np.arange(-MAX_RAMP_PCT, MAX_RAMP_PCT + 1, 1.0)  # 11 candidates, 1% resolution


def safety_limits_from_reference(csv_path, margin_frac=0.20, round_to=5.0):
    """Derive WHP/FLP/BHP safe operating limits from the reference dataset's observed
    range plus headroom, since the challenge brief doesn't specify numeric limits.

    PLACEHOLDER -- DERIVED FROM SAMPLE DATA, NOT OFFICIAL LIMITS. Replace with the
    real operating envelope if/when one is specified.
    """
    df = pd.read_csv(csv_path)
    limits = {}
    for ch, col in [("WHP", "WHP_psi"), ("FLP", "FLP_psi"), ("BHP", "BHP_psi")]:
        lo, hi = df[col].min(), df[col].max()
        margin = margin_frac * (hi - lo)
        lo_lim = round_to * math.floor((lo - margin) / round_to)
        hi_lim = round_to * math.ceil((hi + margin) / round_to)
        limits[ch] = (lo_lim, hi_lim)
    return limits


def _steady_state(p, u):
    y = p["A"] + p["B"] * u / (u + p["uh"])
    return max(0.0, y) if p["clip_nonneg"] else y


def _predict_horizon(model, state, u_history, new_u, horizon, ts):
    """Simulate `horizon` steps ahead: apply new_u this interval, then hold it there.
    Returns {channel: array of length horizon} of predicted values."""
    u_seq = u_history + [new_u] * horizon
    base_len = len(u_history)
    preds = {ch: np.empty(horizon) for ch in model}
    cur = dict(state)
    for k in range(horizon):
        for ch, p in model.items():
            idx = max(0, base_len + k - p["theta_steps"])
            y_ss = _steady_state(p, u_seq[idx])
            alpha = ts / p["tau"]
            cur[ch] = cur[ch] + alpha * (y_ss - cur[ch])
            preds[ch][k] = cur[ch]
    return preds


class MPCController:
    """One-sentence-explainable brute-force MPC: 11 fixed candidate moves, simulated
    over a lookahead horizon, filtered by safety, chosen by closeness to target."""

    def __init__(self, model, limits, ts_hours=1.0, horizon=None, noise_margin_sigma=3.0):
        self.model = model
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
        """state: dict with current Q, WHP, FLP, BHP readings.
        Returns (new_choke_position, one_sentence_explanation)."""
        if not self.u_history:
            self.u_history = [current_choke]

        results = []
        for delta in CANDIDATE_DELTAS:
            new_u = float(np.clip(current_choke + delta, CHOKE_MIN, CHOKE_MAX))
            preds = _predict_horizon(self.model, state, self.u_history, new_u, self.horizon, self.ts)
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
                f"Moved choke to {chosen['new_u']:.0f}% because it keeps WHP/FLP/BHP within "
                f"safe limits over the next {self.horizon}h and brings predicted oil rate to "
                f"{chosen['q_end']:.1f} bbl/hr, closest to the {target_q:.1f} bbl/hr target "
                f"among {len(feasible)} feasible options."
            )
        else:
            chosen = min(results, key=lambda r: (r["violation"], abs(r["new_u"] - current_choke)))
            explanation = (
                f"No choke move keeps all limits satisfied over the lookahead; moved to "
                f"{chosen['new_u']:.0f}% because it minimizes the predicted constraint "
                f"violation ({chosen['violation']:.1f} psi-steps over horizon)."
            )

        self.u_history.append(chosen["new_u"])
        return chosen["new_u"], explanation


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
    new_u, why = ctrl.decide(state, current_choke=30.0, target_q=150.0)
    assert new_u > 30.0, "expected controller to open the choke toward an achievable higher target"
    assert new_u - 30.0 <= MAX_RAMP_PCT + 1e-9, "ramp-rate constraint violated"
    print(f"achievable-target case: {why}")

    impossible_limits = {ch: (lo, lo + 0.01) for ch, (lo, hi) in limits.items()}
    ctrl2 = MPCController(model, impossible_limits)
    new_u2, why2 = ctrl2.decide(state, current_choke=30.0, target_q=150.0)
    assert 0.0 <= new_u2 <= 100.0
    assert abs(new_u2 - 30.0) <= MAX_RAMP_PCT + 1e-9
    print(f"impossible-limits case: {why2}")
    print("controller.py self-check PASSED")


if __name__ == "__main__":
    demo()
