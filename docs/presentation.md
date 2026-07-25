# ChokePilot — Autonomous Choke Controller
### Honeywell Campus Connect Hackathon (PS3)

Speaker notes are in *italics* under each slide.

---

## Section 1 — Process Understanding & Model

---

### Slide 1 — The Problem

Operators and engineers don't under-perform because the physics is hard — they
under-perform because of five well-documented, unglamorous frictions:

1. **Operators "baby" the choke** — fear of sand/formation damage leaves real
   production capacity unused. *(Eng-Tips forum discussion)*
2. **Too many wells, too little attention** — a choke gets set once during a
   shift and left alone, because nobody has the bandwidth to revisit it
   continuously. *(patent literature on standard industry practice)*
3. **Slugging is caught reactively** — a human notices fluctuations after the
   fact, not before. *(SPE technical paper)*
4. **APC doesn't scale upstream because models need a dedicated team** — the
   on-record barrier isn't algorithmic sophistication, it's who keeps the
   model current. *(on-record Honeywell upstream solutions interview)*
5. **Alarm fatigue and distrust of opaque automation** — operators override
   what they don't understand. *(NTSB SCADA safety review)*

*ChokePilot answers all five: continuous optimization instead of a
"set and forget" choke; a search-based, no-heavy-optimizer controller with a
fresh from-scratch identification step instead of a model a dedicated team
must maintain — no solver license, no optimizer internals to tune, nothing
exotic to keep alive; and a logged one-sentence rationale for every move
instead of a black box. ChokePilot is a small-scale, single-well version of
the same closed-loop direction Honeywell's APC customers are already moving
toward on wells — a focused proof-of-concept of the wellhead-to-control-room
optimization problem Honeywell's Upstream Production Performance Suite
(UPPS) addresses at scale. It does not claim equivalence; it echoes the same
idea at a scale one person can build and verify in a project timeline.*

---

### Slide 2 — No Official Simulator: What We Built, and What It Can't Tell Us

- The platform provided no simulator — only
  `Autonomous_Choke_Control_Simulated_Dataset.csv` (120 hr, choke tested
  30–65%) and a presentation template.
- `data/simulator.py` is a **calibrated substitute**, clearly commented as
  such, exposing the exact interface the brief specifies:
  `Q, WHP, FLP, BHP = simulator.step(choke_position)`.
- Steady-state map per channel: `y_ss(u) = A + B·u/(u+uh)`; Q is forced
  through zero at 0% choke (a fully shut choke physically gives zero flow).
- Dynamics: FOPDT, discretized with the **exact** zero-order-hold form
  `alpha = 1 - exp(-Ts/tau)` — not the Euler approximation `Ts/tau`, which is
  only accurate when `Ts ≪ tau` and can go unstable outside that regime.
- The steady-state fit uses **only the last ~6 hours of each dwell period**
  (near-settled samples), not the whole trajectory including transients —
  fitting on transient-contaminated samples was found to bias the curve and
  the downstream τ/θ search that compensates for it.

**Two honestly-reported findings, not hidden:**
- With the `uh` search grid widened to check for boundary-pinning, **WHP,
  FLP, and BHP all pin at the widened boundary (50,000)** — their
  steady-state response is statistically indistinguishable from linear over
  the tested 30–65% range. Only Q, anchored through zero, genuinely needs the
  saturating shape. The fitting code prints this as a warning rather than
  silently accepting it.
- The reference CSV covers **only 5 distinct choke levels**, one dwell period
  each, fit as a single deterministic least-squares point estimate — there is
  no confidence interval or uncertainty quantification anywhere in this
  pipeline. Every downstream number inherits that uncertainty without a
  formal error bar.

---

### Slide 3 — Open-Loop Step Test: What It Actually Tests

- Per the brief, `identify.py` runs its **own** fresh monotonic staircase
  (0→100% in 10% steps, 24h dwell) against the calibrated simulator, rather
  than reusing the reference CSV.
- **Reframing an earlier claim:** `identify.py` imports `_fit_fopdt` and
  `_simulate_fopdt` directly from `data/simulator.py` — the *same* private
  fitting functions used to calibrate the simulator itself, not an
  independently implemented estimator. The functional form is shared **by
  construction**. Calling this "treating the simulator as a black box"
  overstates what's actually being tested.
- What it legitimately tests: **parameter identification accuracy under a
  correctly-specified model.** `verify_identification.py` compares identified
  vs. true τ across 5 seeds (0, 1, 2, 7, 99):

| Channel | True τ (h) | Mean identified τ (h) | Mean error | Error range |
|---|---|---|---|---|
| Q | 5.00 | 4.00 | 20.0% | 15.0–25.0% |
| WHP | 7.50 | 6.30 | 16.0% | 13.3–16.7% |
| FLP | 7.00 | 5.10 | 27.1% | 25.0–28.6% |
| BHP | 9.00 | 6.20 | 31.1% | 25.0–38.9% |

*Identified τ is lower than true τ in all 20/20 seed×channel combinations —
one-directional bias, not scattered noise. Since the model structure is
guaranteed correct, this comes from elsewhere — most plausibly finite dwell
time relative to the (larger) true time constants: BHP's true τ=9h with θ=2h
needs roughly 4τ+θ ≈ 38h to fully settle, well past the 24h dwell used. This
is flagged as a real, unresolved limitation. Dead-time identification is
tighter: Q and FLP match the true θ exactly every seed; WHP and BHP are
consistently off by exactly 1 sample.*

---

### Slide 4 — Hybrid Physics + Learned Correction: A Null Result, Reported Honestly

- A small degree-1 polynomial-in-choke correction on the physics FOPDT
  residual, validated on a **held-out** step test before being trusted.

| Channel | Physics-only RMSE | +Correction RMSE | Kept? |
|---|---|---|---|
| Q | 1.88 | 2.04 | ❌ skipped |
| WHP | 1.33 | 1.42 | ❌ skipped |
| FLP | 1.00 | 1.08 | ❌ skipped |
| BHP | 10.66 | 10.79 | ❌ skipped |

*After the steady-state re-fit and exact-ZOH discretization fixes (Slide 2),
the physics-only fit is now tight enough on every channel that the
correction never generalizes — disabled everywhere by
`select_beneficial_corrections()`. The controller currently runs on pure
physics. The mechanism stays in the codebase — the validate-before-trust
discipline is the actual point of including it, not the RMSE number — in
case a future recalibration reopens a gap it could close.*

---

### Slide 5 — Safety Limits Are One-Sided, Not Symmetric Bands

| Channel | Direction enforced | Limit | Brief basis |
|---|---|---|---|
| WHP | floor only (`hi = +inf`) | ≥ 205 psi | *"If WHP becomes too low, the well may operate outside its recommended operating envelope."* High WHP just means the choke is closed back further — safe. |
| BHP | floor only (`hi = +inf`) | ≥ 2830 psi | *"one of the most important indicators of reservoir health and drawdown"* — low BHP means excessive drawdown; high BHP means safely choked back. |
| FLP | ceiling only (`lo = -inf`) | ≤ 200 psi | *"helps ensure stable transportation of produced fluids"* — the risk is backpressure/separator overpressure on the high side, not a low reading. |

*An earlier draft bracketed all three symmetrically, which could reject a
high-WHP/high-BHP or low-FLP state that isn't actually unsafe. These are
further tightened inside the controller (Slide 7) before use.*

---

## Section 2 — Control Strategy

---

### Slide 6 — What This Actually Is: One-Step Search With Hold-Constant Prediction

*An earlier draft called this "brute-force MPC." That overstates it.*

Each control interval:

1. Enumerate **11 legal choke moves** (−5% to +5% in 1% steps — the full
   ramp-rate range) for **this interval only** — not a search over a
   sequence of future moves.
2. For each candidate, **hold that new position constant** for the rest of
   the lookahead horizon and simulate forward — a cheap proxy for "what
   happens if I stop adjusting," not a genuine multi-step trajectory
   optimization.
3. Reject any candidate predicted to breach a WHP/FLP/BHP limit anywhere in
   that held-constant horizon.
4. Command the feasible candidate whose end-of-horizon predicted oil rate is
   closest to target; if none is feasible, fall back to the one minimizing
   predicted violation.

- This **is** receding-horizon in the classic sense — it re-runs every
  interval with fresh measurements — but the per-step search itself is a
  one-step decision screened by a hold-constant forward simulation.
  Horizon is derived, not hand-tuned: `ceil(3 × max(τ) / Ts)`, clipped to
  [3, 12] hours.
- **Safety here is best-effort, not formally guaranteed** — no recursive
  feasibility check, no terminal invariant set. The 30-seed sweep (Slide 13)
  shows the safety-fallback branch firing on **23.6%** of Scenario C's steps,
  precisely because that scenario runs at the edge of what the model
  considers feasible, with no formal guarantee behind it — only a greedy,
  per-step search that has worked well empirically.

---

### Slide 7 — Constraint Tightening for Noise

- Predictions are noise-free, but real sensor readings carry measurement
  noise — riding the true limit exactly would let real noise dip past it.
- Standard robust-MPC constraint tightening: back off each limit by **3σ**
  of that channel's identified `noise_std` before the controller ever
  compares a prediction against it.
- If no candidate is fully feasible, the controller falls back to the one
  that minimizes predicted violation — it never freezes or refuses to act.

---

### Slide 8 — The Extrapolation Problem Belongs to the Simulator, Not the Controller

*An earlier draft claimed Scenario A's 15% starting choke was "inside the
model's supported range." That claim was false.*

- The reference CSV's tested band is 30–65%; 15% is below it. Moving
  Scenario A's start from 0% to 15% **reduced how far** into extrapolated
  territory the startup transient reaches — it did not eliminate the
  extrapolation.
- Extrapolation uncertainty is a property of the substitute simulator's
  calibration (only 5 discrete choke levels were ever observed, all ≥30% —
  Slide 2), not something the controller can detect or correct for. The
  controller has no way to know its own prediction model is running on
  unvalidated territory; it simply predicts with the curve it has.
- Any confidence below 30% choke is inherited entirely from the fitted
  curve's shape (monotonic and bounded by construction) staying "physically
  sane" under extrapolation — not from evidence.
- Scenario A's residual violations, reported across 30 seeds (Slide 10), are
  the direct, expected consequence.

---

### Slide 9 — Explainability Is a Logged Field, Not a Slide

Directly answers pain point #5: every decision produces a real,
human-readable rationale — a field in the output CSV, in both branches.

> *"Moved choke to 34.0% because it keeps WHP/FLP/BHP within safe limits over
> the next 12h and brings predicted oil rate to 99.1 bbl/hr, closest to the
> 100.0 bbl/hr target among 11 feasible options."*
> — normal branch, actual logged decision

> *"No choke move keeps all limits satisfied over the lookahead; moved to
> 65.4% because it minimizes the predicted constraint violation (5.8
> psi-steps over horizon)."*
> — safety-fallback branch, same field, same guarantee

*Commanded choke values are rounded to 0.1% before being used for
prediction, selection, and the logged string — the rationale can never
silently diverge from what was actually applied.*

**Honeywell echo:** dashboard/output layering mirrors Uniformance's PHD
(historian) / KPI (target-tracking) / Asset Sentinel (risk/alert) structure —
a focused proof-of-concept of the same wellhead-to-control-room optimization
problem UPPS addresses at scale. *(echoes, not replicates.)*

---

## Section 3 — Results

*All single-run numbers below use the default seed; the 30-seed sweep
(Slide 13) is the number to trust over any one run.*

---

### Slide 10 — Scenario A: Startup to Target

- 15% choke → 100 bbl/hr target, 80-hour run.
- **Tracking:** trailing 10-hour mean (hours 70–79) of **99.26 bbl/hr** at a
  steady 34.0% choke.
- **Safety, single seed:** 2/80 constraint samples outside limits.
- **Safety, honest (30-seed sweep):** every seed shows at least one
  violation (30/30), mean **2.67/80**, max 4/80 — a stable, deterministic
  consequence of starting inside extrapolated territory, not one noisy run.
- **Ramp-rate:** 0/80 violations in every seed — the extrapolation-driven
  pressure transient was the binding limit, never the ±5%/interval
  constraint.

---

### Slide 11 — Scenario B: Target Tracking

- 34.4% choke start, target steps **100 → 150 bbl/hr at t = 60h**, 140-hour
  run.
- **Tracking:** trailing 10-hour mean **100.76 bbl/hr** at 34.4% choke before
  the step (hours 50–59); **150.87 bbl/hr** at 61.4% choke after settling
  (hours 130–139).
- **Safety:** 0/140 in the representative run **and** 0/140 in **all 30
  seeds** — the only scenario with a perfectly clean sweep.

---

### Slide 12 — Scenario C: Infeasible Target

- 34.4% choke start, **400 bbl/hr requested** — deliberately beyond what's
  safely achievable.
- **Behavior:** does not chase the target into a violation. Trailing
  10-hour mean **162.80 bbl/hr** at 69.7% choke — the maximum rate the
  tightened envelope allows, not 400.
- **Safety, single seed:** 0/100. **30-seed sweep:** mean 0.07/100, max 1,
  only 2/30 seeds show any violation at all — a rare, small residual, not a
  systematic problem.
- **Safety-fallback frequency:** 23.6% of steps — far higher than A or B,
  because this scenario deliberately runs near the edge of the tightened
  feasible envelope for its entire duration. Frequent fallback here is
  expected behavior, not a red flag.

---

### Slide 13 — Seed-Sweep Distribution (30 seeds per scenario)

| Scenario | Mean violations | Max violations | Seeds with ≥1 violation | Mean safety-fallback rate |
|---|---|---|---|---|
| A — Startup to Target | 2.67 / 80 | 4 | 30 / 30 | 4.38% |
| B — Target Tracking | 0.00 / 140 | 0 | 0 / 30 | 0.00% |
| C — Infeasible Target | 0.07 / 100 | 1 | 2 / 30 | 23.57% |

*Full per-seed results: `outputs/seed_sweep_results.csv`. A single seed's "0
violations" is a point sample from a noisy system — the distribution is what
actually supports a safety claim.*

---

### Slide 14 — Baseline Comparison: MPC vs. Fixed Choke vs. PI

Two baselines, run over identical scenarios/model/limits/seeds:

- **Fixed** — the choke the identified model says holds the target at
  steady state, walked back until its own steady-state predictions clear the
  tightened envelope, then held. Models pain point #2: set once, left alone.
- **PI** — velocity-form PI on oil rate, IMC-tuned from the identified
  model, **blind to WHP/FLP/BHP by design** — the point of the comparison.

| Scenario | Approach | Safety violations | Total barrels | Notes |
|---|---|---|---|---|
| A | MPC | 2/80 | 7681.1 | |
| A | Fixed | 3/80 | 7730.5 | |
| A | PI | 3/80 | 7727.0 | |
| B | MPC | 0/140 | 17759.0 | settling time 73h, overshoot 7.4% |
| B | Fixed | 0/140 | 17748.4 | settling time 48h — faster, no lookahead caution |
| B | PI | 0/140 | 17651.1 | settling time 70h, overshoot 10.1% — worst overshoot |
| C | MPC | **0/100** | 15884.7 | |
| C | Fixed | **0/100** | 16016.9 | |
| C | PI | **169/100** | 18880.5 | blindly chases the infeasible target into repeated violations |

*The PI baseline's 169 safety violations in Scenario C (out of 100 steps,
summed across three pressure channels — so it can exceed 100) is the
clearest result in this whole comparison: a controller with no predictive
safety check chases an infeasible target straight through the operating
envelope. The MPC's one-step lookahead-with-rejection prevents that at
essentially no production cost — 15,884.7 vs. 18,880.5 barrels, and PI only
"produces more" because it isn't stopping at the safety boundary. Full
results: `outputs/baseline_comparison.csv`.*

---

### Slide 15 — Lessons Learned

- **Report the model's real support region, not an aspirational one.** An
  earlier draft claimed Scenario A's 15% start was "inside supported
  territory." It wasn't — the extrapolation problem belongs to the
  simulator's calibration, not the controller, and no amount of controller
  tuning fixes a model guessing outside its fitted region.
- **A correctly-specified model still shows real identification error.**
  Even with the exact right functional form, 16–31% mean τ error persists —
  most likely from finite dwell time relative to the now-larger true time
  constants. A concrete, actionable lead (extend `DWELL_HOURS`), not a vague
  "the model doesn't fit well."
- **Name the controller accurately.** Calling this "brute-force MPC" implied
  guarantees — recursive feasibility, trajectory optimality — it doesn't
  have. It's a one-step receding-horizon search with hold-constant
  prediction: empirically strong on safety, but not formally proven. Both
  things are true; only naming it precisely lets a reader hold both.
- **Report distributions, not single runs.** A single seed's "0 violations"
  or "104.0 bbl/hr" is a point sample. The 30-seed sweep is what actually
  supports a safety claim (Scenario B clean in all 30 seeds; Scenario A
  violates in all 30) or a tracking claim (trailing 10-hour means, not one
  timestamp).
- **A correction layer earns its place by generalizing, or it doesn't
  ship.** The hybrid correction currently helps nowhere, post-fix — reported
  as a null result rather than removed or hidden, since the
  validate-before-trust discipline is the actual point, not the RMSE number.
