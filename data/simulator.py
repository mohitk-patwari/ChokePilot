"""
Calibrated substitute for the official Honeywell single-well choke simulator.

No official simulator was provided on this platform for this challenge -- only the
reference dataset `Autonomous_Choke_Control_Simulated_Dataset.csv` (a single 120-hour
multi-step run: choke held at 30/40/55/45/65 % in turn). This module fits a simple,
physically-motivated model to that dataset and exposes the interface the challenge
specifies:

    Q, WHP, FLP, BHP = simulator.step(choke_position)

Model, per output channel (Q, WHP, FLP, BHP), independently:
  - Steady-state map y_ss(u) = A + B * u / (u + uh)   (saturating in choke opening u,
    the standard shape for both a choke's flow-vs-opening curve and pressure-vs-opening
    curve -- monotonic, bounded, matches Q(0)=0 physically).
  - Dynamics: first-order-plus-dead-time discrete recursion driven by u through y_ss.
  - A small Gaussian noise term matched to the residual std left after the above fit,
    so the simulator's noise level looks like the reference data's.

uh (per channel) and (tau, theta) are found by small brute-force grid search with the
linear part (A, B) solved in closed form at each grid point -- consistent with this
project's "brute-force is fine, don't reach for an optimizer" philosophy.

Swap in the real simulator if one becomes available -- nothing else in this project
(identify.py, controller.py, scenarios.py) depends on this file's internals, only on
Simulator.step().

Known limitation: the reference data only exercises choke positions 30-65 %. Behavior
outside that band (needed for Scenario A startup and Scenario C push-to-max) is an
extrapolation of the fitted saturating curves, not observed data. The curves are
monotonic and bounded by construction so extrapolation stays physically sane, but it is
still a guess -- replace with real data if/when available.
"""

from pathlib import Path

import numpy as np
import pandas as pd

_CSV_PATH = Path(__file__).parent / "Autonomous_Choke_Control_Simulated_Dataset.csv"
TS_HOURS = 1.0  # matches the challenge's control interval

_CHANNEL_COLUMNS = {
    "Q": "OilRate_bbl_hr",
    "WHP": "WHP_psi",
    "FLP": "FLP_psi",
    "BHP": "BHP_psi",
}


# uh grid: fine resolution 1-300 (original range) then log-spaced out to 50,000 so a
# channel whose data is genuinely close to linear over the tested range (uh >> u,
# i.e. u/(u+uh) ~ u/uh) has somewhere to go instead of pinning at an arbitrary cap.
_UH_GRID = np.concatenate([np.linspace(1, 300, 300), np.geomspace(300, 50_000, 200)])


def _steady_state_samples(u, y, tail_samples):
    """Return only the last `tail_samples` of each constant-u dwell period -- the
    samples closest to actually being at steady state -- so the steady-state map is
    fit without transient contamination. Detects dwell boundaries generically from
    wherever u changes, not from a hardcoded schedule."""
    change_idx = np.flatnonzero(np.diff(u) != 0) + 1
    seg_bounds = np.concatenate([[0], change_idx, [len(u)]])
    mask = np.zeros(len(u), dtype=bool)
    for s, e in zip(seg_bounds[:-1], seg_bounds[1:]):
        mask[max(s, e - tail_samples):e] = True
    return u[mask], y[mask]


def _fit_saturating(u, y, force_zero_at_u0=False, tail_samples=6, label=None):
    """Least-squares fit of y = A + B * u / (u + uh) (or, if force_zero_at_u0, the
    pure Michaelis-Menten form y = B * u / (u + uh), A fixed at 0 -- for oil rate,
    where a fully shut choke must give zero flow, not an extrapolated intercept).

    Fit only on the last `tail_samples` of each dwell period (steady-state samples),
    not the whole trajectory -- otherwise transient (not-yet-settled) samples get
    treated as if they were steady-state, biasing the fitted curve (and, downstream,
    the dynamics fit that compensates for it -- see verify_identification.py).

    uh is found by 1-D grid search; A, B follow in closed form (ordinary linear
    regression) for each uh candidate -- a small brute-force + closed-form fit, no
    optimizer dependency needed.
    """
    u_ss, y_ss = _steady_state_samples(u, y, tail_samples)
    best = None
    for uh in _UH_GRID:
        x = u_ss / (u_ss + uh)
        if force_zero_at_u0:
            b = float(np.dot(x, y_ss) / np.dot(x, x))
            a = 0.0
            resid = y_ss - b * x
        else:
            design = np.column_stack([np.ones_like(x), x])
            coef, *_ = np.linalg.lstsq(design, y_ss, rcond=None)
            a, b = coef
            resid = y_ss - design @ coef
        sse = float(resid @ resid)
        if best is None or sse < best[0]:
            best = (sse, a, b, uh)
    _, a, b, uh = best
    if uh >= 0.98 * _UH_GRID.max():
        tag = f" ({label})" if label else ""
        print(f"WARNING: uh{tag} pinned at grid boundary ({uh:.0f} >= 98% of "
              f"{_UH_GRID.max():.0f}) -- uh is unidentifiable from this data; the "
              f"steady-state response is statistically indistinguishable from linear "
              f"over the tested choke range, not a resolvable saturation curve.")
    return a, b, uh


def _simulate_fopdt(u, y0, a, b, uh, tau, theta_steps, ts, clip_nonneg=False):
    u_delayed = np.concatenate([np.full(theta_steps, u[0]), u])[: len(u)]
    y_ss = a + b * u_delayed / (u_delayed + uh)
    if clip_nonneg:
        y_ss = np.maximum(y_ss, 0.0)
    sim = np.empty(len(u))
    sim[0] = y0
    # Exact zero-order-hold discretization of dy/dt = (y_ss - y)/tau over one sample
    # interval with y_ss held constant during it -- unconditionally stable for any
    # ts/tau ratio and exact (not a first-order approximation like alpha = ts/tau).
    alpha = 1.0 - np.exp(-ts / tau)
    for k in range(len(u) - 1):
        sim[k + 1] = sim[k] + alpha * (y_ss[k] - sim[k])
    return sim


def _fit_fopdt(u, y, ts, clip_nonneg=False, label=None):
    """Grid-search tau (time constant, hours) and theta (dead time, whole samples)
    minimizing SSE between the recorded trajectory and the FOPDT recursion's."""
    a, b, uh = _fit_saturating(u, y, force_zero_at_u0=clip_nonneg, label=label)
    best = None
    for theta_steps in range(0, 4):
        for tau in np.linspace(0.5, 20.0, 79):
            sim = _simulate_fopdt(u, y[0], a, b, uh, tau, theta_steps, ts, clip_nonneg)
            sse = float(np.sum((sim - y) ** 2))
            if best is None or sse < best[0]:
                best = (sse, tau, theta_steps)
    _, tau, theta_steps = best
    resid = y - _simulate_fopdt(u, y[0], a, b, uh, tau, theta_steps, ts, clip_nonneg)
    return {
        "A": a, "B": b, "uh": uh, "tau": tau, "theta_steps": theta_steps,
        "noise_std": float(np.std(resid)), "clip_nonneg": clip_nonneg,
    }


def _calibrate():
    df = pd.read_csv(_CSV_PATH)
    u_hist = df["Choke_pct"].to_numpy(dtype=float)
    params = {}
    for ch, col in _CHANNEL_COLUMNS.items():
        y_hist = df[col].to_numpy(dtype=float)
        params[ch] = _fit_fopdt(u_hist, y_hist, TS_HOURS, clip_nonneg=(ch == "Q"), label=ch)
    return params


PARAMS = _calibrate()  # calibrated once at import time, ~1s


def steady_state_from_params(p, u, correction_coefs=None):
    """The one canonical y_ss(u) formula: physics steady-state map, optionally plus a
    small learned correction(u) (see identify.py's fit_residual_correction).
    correction_coefs=None means physics-only. Single shared implementation --
    imported by controller.py and identify.py -- so there's exactly one place that
    can disagree with itself about whether/how correction is applied."""
    y = p["A"] + p["B"] * u / (u + p["uh"])
    if correction_coefs is not None:
        y += float(np.polyval(correction_coefs, u))
    return max(0.0, y) if p["clip_nonneg"] else y


def steady_state(channel, u):
    """y_ss(u) for one channel of the plant's OWN calibration -- physics-only by
    design (the true plant has no "learned correction", that's a model concept)."""
    return steady_state_from_params(PARAMS[channel], u)


class Simulator:
    """Stateful single-well choke simulator. Call .step(choke_position) once per
    control interval (Ts = 1 hour); returns (Q, WHP, FLP, BHP) for that interval.

    The FOPDT recursion evolves a clean internal "true" state; each returned reading
    adds independent Gaussian measurement noise (calibrated to the reference data's
    residual level) without feeding it back into the recursion -- sensor noise, not
    process noise, so it doesn't randomly accumulate step over step.

    Ramp-rate (+-5 %/interval) is NOT enforced here -- that is the controller's
    responsibility, matching how a real actuator will accept whatever setpoint it's
    given while the control logic is what's supposed to behave safely.
    """

    def __init__(self, initial_choke=0.0, seed=None):
        self._rng = np.random.default_rng(seed)
        self.reset(initial_choke)

    def reset(self, initial_choke=0.0):
        u0 = float(np.clip(initial_choke, 0.0, 100.0))
        self._u_history = [u0]
        self._true_state = {ch: steady_state(ch, u0) for ch in PARAMS}
        return self._read()

    def _read(self):
        return tuple(
            self._true_state[ch] + self._rng.normal(0.0, PARAMS[ch]["noise_std"])
            for ch in ("Q", "WHP", "FLP", "BHP")
        )

    def step(self, choke_position):
        u = float(np.clip(choke_position, 0.0, 100.0))
        self._u_history.append(u)
        for ch, p in PARAMS.items():
            idx = max(0, len(self._u_history) - 1 - p["theta_steps"])
            u_eff = self._u_history[idx]
            y_ss = steady_state(ch, u_eff)
            alpha = 1.0 - np.exp(-TS_HOURS / p["tau"])  # exact ZOH, see _simulate_fopdt
            self._true_state[ch] = self._true_state[ch] + alpha * (y_ss - self._true_state[ch])
        return self._read()


simulator = Simulator()  # default shared instance matching the challenge's example interface


def demo():
    """Self-check: does each channel's calibrated FOPDT fit track the reference CSV?

    Compares the *deterministic* recursion (no injected measurement noise) against the
    real trajectory. Injected noise is calibrated to match the data's own noise level
    by construction, so folding it into this comparison would just add an irreducible
    ~noise_std-scale floor unrelated to calibration quality -- this checks the thing
    that's actually tunable: the steady-state map and FOPDT dynamics.
    """
    df = pd.read_csv(_CSV_PATH)
    u_hist = df["Choke_pct"].to_numpy(dtype=float)
    for ch, col in _CHANNEL_COLUMNS.items():
        p = PARAMS[ch]
        actual = df[col].to_numpy(dtype=float)
        sim = _simulate_fopdt(u_hist, actual[0], p["A"], p["B"], p["uh"], p["tau"],
                               p["theta_steps"], TS_HOURS, clip_nonneg=p["clip_nonneg"])
        rmse = float(np.sqrt(np.mean((sim - actual) ** 2)))
        band = actual.max() - actual.min()
        assert rmse < 0.15 * band, f"{ch}: RMSE {rmse:.2f} too high vs data band {band:.2f}"
        print(f"{ch}: deterministic RMSE={rmse:.3f}  (data band={band:.1f}, "
              f"tau={p['tau']:.2f}h, theta={p['theta_steps']}h, noise_std={p['noise_std']:.3f})")
    print("simulator.py self-check PASSED (deterministic fit within 15% of data range, per channel)")


if __name__ == "__main__":
    demo()
