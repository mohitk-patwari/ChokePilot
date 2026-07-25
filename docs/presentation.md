# ChokePilot — Autonomous Choke Controller
### Honeywell Campus Connect Hackathon (PS3)

Speaker notes are in *italics* under each slide. Numbers below are pulled
directly from the current `docs/report.md` (post `DWELL_HOURS=40` fix) —
"§N" references point back there for the full derivation.

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
must maintain; and a logged one-sentence rationale for every move instead of
a black box. It's a small-scale, single-well version of the same closed-loop
direction Honeywell's APC customers are already moving toward on wells — a
focused proof-of-concept of the wellhead-to-control-room optimization problem
Honeywell's Upstream Production Performance Suite (UPPS) addresses at scale.
It does not claim equivalence; it echoes the same idea at a scale one person
can build and verify in a project timeline. (report.md §0)*

---

### Slide 2 — No Official Simulator: What We Built, and What It Can't Tell Us *(report.md §1.1)*

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
  (near-settled samples), not the whole trajectory — fitting on
  transient-contaminated samples was found to bias the curve and the
  downstream τ/θ search that compensates for it.

**Two honestly-reported findings, not hidden:**
- With the `uh` search grid widened to check for boundary-pinning, **WHP,
  FLP, and BHP all pin at the widened boundary** — their steady-state
  response is statistically indistinguishable from linear over the tested
  30–65% range. Only Q, anchored through zero, genuinely needs the
  saturating shape.
- The reference CSV covers **only 5 distinct choke levels**, fit as a single
  deterministic least-squares point estimate — no confidence interval or
  uncertainty quantification anywhere in this pipeline.

---

### Slide 3 — Open-Loop Step Test: What It Actually Tests *(report.md §1.2)*

- Per the brief, `identify.py` runs its **own** fresh monotonic staircase
  (0→100% in 10% steps) against the calibrated simulator, rather than
  reusing the reference CSV.
- **Reframing an earlier claim:** `identify.py` imports `_fit_fopdt` and
  `_simulate_fopdt` directly from `data/simulator.py` — the *same* private
  fitting functions used to calibrate the simulator itself. The functional
  form is shared **by construction**. Calling this "treating the simulator
  as a black box" overstates what's actually being tested.
- What it legitimately tests: **parameter identification accuracy under a
  correctly-specified model.** `verify_identification.py` compares identified
  vs. true τ across 5 seeds (0, 1, 2, 7, 99):

| Channel | True τ (h) | Mean identified τ (h) | Mean error | Error range |
|---|---|---|---|---|
| Q | 5.00 | 4.05 | 19.0% | 15.0–25.0% |
| WHP | 7.50 | 7.20 | 4.0% | 0.0–6.7% |
| FLP | 7.00 | 6.05 | 13.6% | 10.7–17.9% |
| BHP | 9.00 | 8.30 | 7.8% | 2.8–11.1% |

*Dead-time identification is tight throughout: Q and FLP match the true θ
exactly every seed; WHP and BHP are consistently off by exactly 1 sample —
unaffected by anything on the next slide.*

---

### Slide 4 — The DWELL_HOURS Investigation: A Confirmed Fix, and a Genuine Open Question *(report.md §1.2, §3.6)*

*This is a finding, not a flaw report — the experiment answered a real
question and the answer wasn't uniform, which is more informative than if it
had been.*

- **Before:** with a 24h dwell per step, mean τ error was Q 20% / WHP 16% /
  FLP 27% / BHP 31%, identified τ lower than true τ in **all 20/20**
  seed×channel combinations — a one-directional bias consistent with
  insufficient settling time (BHP's true τ=9h, θ=2h needs ~4τ+θ ≈ 38h to
  settle, past a 24h dwell).
- **The fix, tested and confirmed, not just hypothesized:** raising
  `DWELL_HOURS` 24→40 dropped WHP's error 16.0%→4.0%, FLP's 27.1%→13.6%,
  BHP's 31.1%→7.8%.
- **Q did not move** (20.0%→19.0%, within noise) — across every dwell length
  tested, 24h through 80h. This experiment **ruled dwell time out** as Q's
  cause rather than leaving it lumped in with the other three's explanation.
- **Leading hypothesis:** Q is the one channel fit with `force_zero_at_u0`
  (a fully shut choke must give zero flow) — a different, more constrained
  curve-fitting problem than the other three. Plausible, not yet proven; kept
  as an open question rather than dressed up as solved.

---

### Slide 5 — Hybrid Physics + Learned Correction: Only BHP Earns Its Place *(report.md §1.3)*

- A small degree-1 polynomial-in-choke correction on the physics FOPDT
  residual, validated on a **held-out** step test before being trusted.

| Channel | Physics-only RMSE | +Correction RMSE | Kept? |
|---|---|---|---|
| Q | 1.85 | 1.90 | ❌ skipped |
| WHP | 1.30 | 1.35 | ❌ skipped |
| FLP | 0.96 | 0.97 | ❌ skipped |
| BHP | 9.84 | 9.80 | ✅ **used** |

*After the steady-state re-fit, ZOH, and 40h-dwell fixes, the physics-only
fit is tight enough on Q/WHP/FLP that the correction never generalizes for
them. BHP is the exception: real residual structure remains, so the
correction earns its place there — the only channel actually running
physics+correction in the controller. `select_beneficial_corrections()`
makes this decision automatically per channel from held-out RMSE alone,
nothing hand-picked. BHP is also the channel with the largest remaining τ
error (Slide 3) — plausibly the correction is partly absorbing that
residual dynamics mismatch; this pipeline can't distinguish the two.*

---

### Slide 6 — Safety Limits Are One-Sided, Not Symmetric Bands *(report.md §1.4)*

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

### Slide 7 — What This Actually Is: One-Step Receding-Horizon Search, Not Full MPC *(report.md §2.1, §2.2)*

*An earlier draft called this "brute-force MPC." That overstates it.*

Each control interval:

1. Enumerate **11 legal choke moves** (−5% to +5% in 1% steps) for **this
   interval only** — not a search over a sequence of future moves.
2. For each candidate, **hold that new position constant** for the rest of
   the lookahead horizon and simulate forward — a cheap proxy for "what
   happens if I stop adjusting," not a genuine multi-step trajectory
   optimization.
3. Reject any candidate predicted to breach a WHP/FLP/BHP limit anywhere in
   that held-constant horizon. Limits are backed off by **3σ** of each
   channel's identified `noise_std` first (standard robust-MPC constraint
   tightening) — noise-free predictions would otherwise ride the true limit
   and let real sensor noise breach it.
4. Command the feasible candidate whose end-of-horizon predicted oil rate is
   closest to target; if none is feasible, fall back to the one minimizing
   predicted violation — the controller never freezes or refuses to act.

- This **is** receding-horizon in the classic sense — it re-runs every
  interval with fresh measurements — but the per-step search itself is a
  one-step decision screened by a hold-constant forward simulation. Horizon
  is derived, not hand-tuned: `ceil(3 × max(τ) / Ts)`, clipped to [3, 12] h.
- **Safety here is best-effort, not formally guaranteed** — no recursive
  feasibility check, no terminal invariant set. The 30-seed sweep (Slide 12)
  shows the safety-fallback branch firing on **15.9%** of Scenario C's steps
  — precisely because that scenario runs at the edge of what the model
  considers feasible, with no formal guarantee behind it, only a greedy
  per-step search that has worked well empirically.

---

### Slide 8 — Explainability Is a Logged Field, Not a Slide *(report.md §2.4)*

Directly answers pain point #5: every decision produces a real,
human-readable rationale — a field in the output CSV, in both branches.

> *"Moved choke to 34.0% because it keeps WHP/FLP/BHP within safe limits over
> the next 12h and brings predicted oil rate to 99.4 bbl/hr, closest to the
> 100.0 bbl/hr target among 11 feasible options."*
> — normal branch, actual logged decision

> *"No choke move keeps all limits satisfied over the lookahead; moved to
> 62.2% because it minimizes the predicted constraint violation (6.8
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

*Single-run numbers below use the default seed; the 30-seed sweep (Slide 12)
is the number to trust over any one run.*

---

### Slide 9 — Scenario A: Startup to Target *(report.md §2.3, §3.1)*

- 15% choke → 100 bbl/hr target, 80-hour run.
- **Tracking:** trailing 10-hour mean (hours 70–79) of **99.26 bbl/hr** at a
  steady 34.0% choke.
- **Safety, single seed:** 2/80. **30-seed sweep:** every seed shows at
  least one violation (**30/30**), mean **2.67/80**, max 4/80.
- **Ramp-rate:** 0/80 violations in every seed.
- **Why this doesn't go to zero:** the reference CSV's tested band is
  30–65%; 15% is below it. Starting at 15% *reduces* how far into
  extrapolated territory the startup transient reaches — it does not
  *eliminate* the extrapolation. This is a property of the substitute
  simulator's calibration (only 5 discrete choke levels were ever observed,
  all ≥30%), not something the controller can detect or correct for, and
  it's **unchanged by the `DWELL_HOURS` fix** (Slide 4) — identification
  accuracy doesn't change what choke region the calibration itself covers.
  An earlier draft claimed 15% was "inside supported territory"; that claim
  was false and is corrected here.

---

### Slide 10 — Scenario B: Target Tracking *(report.md §3.2)*

- 34.2% choke start, target steps **100 → 150 bbl/hr at t = 60h**, 140-hour
  run.
- **Tracking:** trailing 10-hour mean **100.32 bbl/hr** at 34.2% choke
  before the step (hours 50–59); **150.57 bbl/hr** at 61.2% choke after
  settling (hours 130–139).
- **Safety:** 0/140 in the representative run **and** 0/140 in **all 30
  seeds** — the only scenario with a perfectly clean sweep, because it never
  leaves the calibration's supported 30–65% band.

---

### Slide 11 — Scenario C: Infeasible Target *(report.md §3.3)*

- 34.2% choke start, **400 bbl/hr requested** — deliberately beyond what's
  safely achievable.
- **Behavior:** does not chase the target into a violation. Trailing
  10-hour mean **162.21 bbl/hr** at 68.9% choke — the maximum rate the
  tightened envelope allows, not 400.
- **Safety:** 0/100 single-seed, **and 0/100 in all 30 seeds** — fully
  clean, an improvement over an earlier (pre-40h-dwell) sweep that showed a
  rare 1-violation residual in 2/30 seeds. Better-identified `noise_std`
  tightens the safety margin more accurately.
- **Safety-fallback frequency:** 15.9% of steps — far higher than A or B,
  because this scenario deliberately runs the choke near the edge of the
  tightened feasible envelope for its entire duration. Frequent fallback
  here is expected behavior, not a red flag; it dropped from an earlier
  23.6% alongside the tighter identified model.

---

### Slide 12 — Seed-Sweep Distribution: 30 Seeds per Scenario *(report.md §3.4)*

| Scenario | Mean violations | Max violations | Seeds with ≥1 violation | Mean safety-fallback rate |
|---|---|---|---|---|
| A — Startup to Target | 2.67 / 80 | 4 | 30 / 30 | 4.67% |
| B — Target Tracking | 0.00 / 140 | 0 | 0 / 30 | 0.00% |
| C — Infeasible Target | 0.00 / 100 | 0 | 0 / 30 | 15.90% |

*Full per-seed results: `outputs/seed_sweep_results.csv`. A single seed's "0
violations" is a point sample from a noisy system — this distribution, not
any one run, is what actually supports a safety claim.*

---

### Slide 13 — Baseline Comparison: MPC vs. Fixed Choke vs. PI *(report.md §3.5 — the strongest single result in this project)*

![Baseline comparison: constraint violations (log scale) and total barrels produced, MPC vs. Fixed vs. PI, per scenario](../outputs/baseline_comparison.png)

- **Fixed** — the choke the identified model says holds the target at
  steady state, walked back until its own steady-state predictions clear
  the tightened envelope, then held. Models pain point #2: set once, left
  alone.
- **PI** — velocity-form PI on oil rate, IMC-tuned from the identified
  model, **blind to WHP/FLP/BHP by design** — the point of the comparison.

| Scenario | Approach | Safety violations | Total barrels | Notes |
|---|---|---|---|---|
| A | MPC | 2/80 | 7,674.5 | |
| A | Fixed | 3/80 | 7,706.6 | |
| A | PI | 3/80 | 7,714.2 | |
| B | MPC | 0/140 | 17,707.7 | settling time 73h, overshoot 6.8% |
| B | Fixed | 0/140 | 17,691.4 | settling time 29h — faster, no lookahead caution |
| B | PI | 0/140 | 17,636.2 | settling time 70h, overshoot 9.3% — worst overshoot |
| C | MPC | **0/100** | 15,833.4 | |
| C | Fixed | **0/100** | 15,831.4 | |
| C | PI | **169/100** | 18,876.2 | blindly chases the infeasible target into repeated violations |

*The PI baseline's **169 safety violations** in Scenario C (out of 100 steps,
summed across three pressure channels — so it can exceed 100) is the single
clearest result in this whole project: a controller with no predictive
safety check chases an infeasible target straight through the operating
envelope. The MPC's one-step lookahead-with-rejection prevents that at
essentially no production cost — 15,833.4 vs. 18,876.2 barrels, and PI only
"produces more" because it isn't stopping at the safety boundary. Every bar
on the left panel above tells this same story at a glance.*

---

### Slide 14 — Lessons Learned *(report.md §3.6)*

- **Report the model's real support region, not an aspirational one.** An
  earlier draft claimed Scenario A's 15% start was "inside supported
  territory." It wasn't — the extrapolation problem belongs to the
  simulator's calibration, not the controller, and no amount of controller
  tuning fixes a model guessing outside its fitted region.
- **A correctly-specified model still shows real identification error — and
  the dwell-time lead was worth chasing down, not just naming.** Raising
  `DWELL_HOURS` 24→40 confirmed the diagnosis for three of four channels
  (WHP 16.0%→4.0%, FLP 27.1%→13.6%, BHP 31.1%→7.8%) and, just as usefully,
  *ruled it out* for Q (20.0%→19.0%, unchanged across 24–80h dwell tested)
  — leaving Q's bias correctly reported as a separate, still-open question
  rather than folded into an explanation that doesn't actually cover it.
- **Name the controller accurately.** Calling this "brute-force MPC" implied
  guarantees — recursive feasibility, trajectory optimality — it doesn't
  have. It's a one-step receding-horizon search with hold-constant
  prediction: empirically strong on safety, but not formally proven. Both
  things are true; only naming it precisely lets a reader hold both.
- **Report distributions, not single runs.** A single seed's "0 violations"
  or "104.0 bbl/hr" is a point sample. The 30-seed sweep is what actually
  supports a safety claim (Scenario B clean in all 30 seeds; Scenario C now
  clean in all 30 too) or a tracking claim (trailing 10-hour means, not one
  timestamp).
- **A correction layer earns its place by generalizing, or it doesn't ship
  — channel by channel, not as a blanket decision.** The hybrid correction
  helps exactly one of four channels (BHP) post-fix, and is used only
  there. Both outcomes (3 skipped, 1 used) come from the same held-out-RMSE
  rule with no manual override — the discipline is the point, not any one
  channel's verdict.
