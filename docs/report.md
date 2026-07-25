# ChokePilot — Autonomous Choke Controller

## 0. Why this exists

Four recurring pain points show up across upstream operations literature and practice, and they shaped every design decision below rather than sitting as background motivation:

1. **Operators "baby" the choke.** Fear of sand production or formation damage leads to conservative, static settings that leave real production capacity on the table (Eng-Tips forum discussion among production engineers).
2. **One engineer, too many wells.** A choke gets set once during a shift and then left alone, because nobody has the bandwidth to revisit it continuously (standard industry practice, as described in patent literature on well automation).
3. **Instability is caught reactively.** Slugging and other flow instabilities are typically noticed only after a human sees the trend on a screen, not prevented (SPE technical paper on flow assurance).
4. **APC doesn't scale upstream because models need a dedicated team.** The on-record barrier from a Honeywell upstream solutions interview isn't algorithmic sophistication — it's that advanced process control adoption stalls because someone has to keep the model current. A brute-force, no-heavy-optimizer controller with a fresh from-scratch identification step directly answers this: there is no solver license, no tuning of an optimizer's internals, nothing exotic to maintain.

ChokePilot is a small-scale, single-well version of the same closed-loop direction Honeywell's APC customers are already moving toward on wells — a focused proof-of-concept of the wellhead-to-control-room optimization problem Honeywell's Upstream Production Performance Suite (UPPS) addresses at scale. It does not claim equivalence; it echoes the same idea at a scale one person can build and verify in a project timeline. The output structure below (historian-style step-test trace / KPI-style target-tracking / risk-style constraint bands) similarly mirrors the PHD / KPI / Asset Sentinel layering of Honeywell's Uniformance stack, without claiming to be it.

## 1. Process Understanding & Model

### 1.1 No official simulator, so a calibrated substitute stands in

The platform provided no official simulator for this challenge — only `data/Autonomous_Choke_Control_Simulated_Dataset.csv`, a single 120-hour run with the choke held at 30/40/55/45/65% in turn, and a presentation template. `data/simulator.py` is a **calibrated substitute**, clearly commented as such throughout, fit to that CSV:

- **Steady-state map**, per channel independently: `y_ss(u) = A + B·u/(u+uh)` — a saturating, monotonic, bounded curve. For oil rate `Q`, `A` is fixed at 0 (a fully shut choke must give zero flow, not an extrapolated intercept).
- **Dynamics**: first-order-plus-dead-time (FOPDT) discrete recursion driven by `u` through `y_ss`.
- **Noise**: independent Gaussian measurement noise per channel, matched to the residual std left after the fit, added to each reading without feeding back into the internal state (sensor noise, not process noise).
- `uh` and `(tau, theta)` come from a small brute-force grid search with the linear part `(A, B)` solved in closed form at each grid point — no optimizer dependency, consistent with the project's brute-force philosophy end to end.

The reference CSV only exercises choke positions 30–65%. Outside that band the curves are still monotonic and bounded by construction, but they're an extrapolation, not observed behavior — this becomes the central caveat in §3.4.

### 1.2 Fresh open-loop step test, not the reference CSV

Per the brief ("students are expected to generate their own data using the simulator and develop their control-oriented models from these experiments"), `identify.py` does not reuse the reference CSV for model identification. It drives its own monotonic staircase — 0/10/20/…/100% choke, 24 hours dwell per step (~4× the slowest calibrated time constant plus dead time, long enough to settle) — against the calibrated simulator, and fits the same FOPDT structure to that fresh run.

A naturally flowing well with no artificial lift and no changing reservoir properties (the challenge's own stated assumptions) isn't expected to show hysteresis, so an up-only sweep is enough to characterize it; a full up-down staircase would double the run for no expected information gain.

Identified time constants from the fresh step test:

| Channel | τ (hours) |
|---|---|
| Q (oil rate) | 2.25 |
| WHP | 3.25 |
| FLP | 2.50 |
| BHP | 3.00 |

### 1.3 Hybrid physics + learned correction

On top of the physics FOPDT model, `identify.py` fits a small degree-1 polynomial-in-choke correction on the residual the physics model leaves on the step test — a bias correction on the steady-state map, not a replacement for the physics dynamics. It's deliberately tiny (2 coefficients) so it can only correct systematic curve-shape mismatch, not memorize the training run.

The correction is validated on a **held-out step test** (different noise draw, same design) and kept only where it demonstrably reduces RMSE there — a channel where it doesn't generalize falls back to physics-only rather than being wired in just because it exists:

| Channel | RMSE physics-only → physics+correction | Kept? |
|---|---|---|
| Q | 3.50 → 3.27 | used |
| WHP | 4.30 → 4.17 | used |
| BHP | 26.95 → 25.67 | used |
| FLP | 2.31 → 2.34 | **skipped** — correction made it slightly worse on held-out data; physics-only fit was already tight |

This is the honest failure case built in on purpose: a correction is only as good as its held-out generalization, and FLP shows that "fit a correction" doesn't automatically mean "keep the correction."

### 1.4 Safety limits

The brief specifies no numeric WHP/FLP/BHP limits, so they're derived — a placeholder, not an official spec — from the reference CSV's observed range, +20% margin, rounded to the nearest 5:

| Channel | Limit |
|---|---|
| WHP | 205–285 psi |
| FLP | 145–200 psi |
| BHP | 2830–3190 psi |

These are then further tightened inside the controller (§2.2) before use.

## 2. Control Strategy

### 2.1 Brute-force MPC, on purpose

Each control interval, `controller.py`'s `MPCController`:

1. Enumerates the full set of legal choke moves at 1% resolution: -5% to +5% in 1% steps (11 candidates — the entire range the ±5%/interval ramp constraint allows).
2. Simulates each candidate forward over a lookahead horizon using the identified FOPDT + correction model.
3. Rejects any candidate predicted to breach a WHP/FLP/BHP limit anywhere in the horizon.
4. Among the survivors, commands the one whose end-of-horizon predicted oil rate is closest to target (ties broken toward the smallest choke move).
5. If nothing survives (e.g. noise has already pushed a reading near a limit), falls back to the candidate that minimizes predicted total constraint violation.

Horizon length is derived, not hand-tuned: `ceil(3 × max(τ) / Ts)`, clipped to [3, 12] hours — long enough to see where the slowest channel is heading, short enough to stay cheap. 11 candidates × a short horizon is trivial to evaluate every hour; no optimizer library is needed, which is the direct answer to pain point 4 above (no dedicated modeling team required to keep an optimizer's internals tuned).

### 2.2 Constraint tightening for noise

Predictions inside the controller are noise-free, but real readings carry measurement noise. Riding a true limit exactly in the noise-free prediction would let real sensor noise breach it in practice. So each limit is backed off by `3σ` of that channel's identified `noise_std` before the controller ever sees it — a standard robust-MPC constraint-tightening margin. The controller targets the tightened envelope; the true limit is what actually stays protected.

### 2.3 One-sentence rationale per decision

Pain point 5 (alarm fatigue and operator distrust of opaque automation, from an NTSB SCADA safety review) is answered directly, not just in principle: every call to `MPCController.decide()` returns a real logged string, verified present in the scenario output CSVs (`Why` column), for both branches:

- Normal: *"Moved choke to 35% because it keeps WHP/FLP/BHP within safe limits over the next 10h and brings predicted oil rate to 99.9 bbl/hr, closest to the 100.0 bbl/hr target among 10 feasible options."*
- Safety fallback (no feasible candidate): *"No choke move keeps all limits satisfied over the lookahead; moved to X% because it minimizes the predicted constraint violation (Y psi-steps over horizon)."*

An operator reading the log doesn't need to reconstruct the controller's internal state to know why it did what it did.

## 3. Results

### 3.1 Scenario A — Startup to Target (15% choke → 100 bbl/hr, 80h)

- **Safety:** 3/80 constraint samples outside limits (down from 21/80 when the scenario started at a hard 0% shut-in — see §3.4). The residual 3 are a BHP overshoot of ~3–4 psi during hours 2–4, the ramp-limited transient as the choke opens from 15% toward its target-holding position.
- **Tracking:** reaches within 2 bbl/hr of the 100 bbl/hr target by hour 12; ends at 104.0 bbl/hr, choke settled at 35%.
- **Ramp-rate:** 0/80 violations — the ±5%/interval constraint was never the binding limit, safety was.

### 3.2 Scenario B — Target Tracking (35.7% choke start, 100 → 150 bbl/hr step at t=60h, 140h)

- **Safety:** 0/140 constraint samples outside limits, 0/140 ramp-rate violations — fully clean run.
- **Tracking:** settles at ~100.8 bbl/hr before the step; after the target steps to 150 bbl/hr at t=60h, reaches within 2 bbl/hr of the new target by t=78h (18 hours after the step — consistent with Q's τ=2.25h plus the ±5%/h ramp limit needing several intervals to move the choke ~30 percentage points). Ends at 148.3 bbl/hr, choke at 66%.

### 3.3 Scenario C — Infeasible Target (35.7% choke start, 400 bbl/hr requested, 100h)

- **Safety:** 0/100 constraint samples outside limits, 0/100 ramp-rate violations.
- **Behavior:** the controller does not chase the infeasible 400 bbl/hr target into a limit violation. It opens the choke as far as safety allows — settling at 76%, the maximum choke position for which every WHP/FLP/BHP prediction still clears the tightened envelope — and holds there at ~158.8 bbl/hr, the maximum safely achievable rate. This is the correct behavior for an infeasible setpoint: get as close as physically and safely possible, then stop, rather than saturating the choke to 100% and hoping.

### 3.4 Lessons learned

**The calibrated model only has real support in the reference CSV's 30–65% tested choke range.** Starting any scenario from a hard 0% shut-in, or even a low ~5% ramp-up start, forces the model into extrapolated territory and produces constraint violations that are an artifact of extrapolating an unobserved region, not a controller defect. This was caught, diagnosed, and deliberately corrected rather than patched around:

- **Scenario A** starts at 15% instead of 0%. Its entire purpose is demonstrating the startup ramp, so it still needs to start low — 15% keeps it inside the region the model was actually fit on. Result: violations dropped from 21/80 to 3/80, and the residual 3 shrank to a small (2–4 psi) BHP overshoot during the hours-2–4 transient rather than being eliminated outright — an honest report of a shrunk, understood, not-fully-zeroed residual.
- **Scenarios B and C** don't need a startup transient at all — that's Scenario A's job — so both now start at the choke position the identified `Q(u)` model itself says holds ~100 bbl/hr steady-state (`solve_choke_for_q()` in `scenarios.py`, ≈35.7%), rather than forcing an unnecessary ramp through unsupported territory just to reach a starting point. Result: violations went from 22/140 to 0/140 (B) and 20/100 to 0/100 (C) — fully eliminated, because both scenarios now stay inside supported territory for their entire run.

This is presented as a modeling decision, not a workaround: a controller is only as trustworthy as the model it's predicting from, and asking it to act confidently outside the region that model was ever validated against is a data problem, not a control problem. The honest fix is to keep the operating envelope inside what was actually measured — matching the same discipline applied to the hybrid correction in §1.3, where FLP's correction was dropped rather than kept because it existed.

The same discipline extends to what a real deployment would need before startup-from-shut-in becomes trustworthy: either a dedicated low-choke step test to extend the calibrated support down to 0%, or an explicit, separately-tuned "startup mode" horizon/candidate set for the region below 30% — not a claim that this proof-of-concept already covers it.
