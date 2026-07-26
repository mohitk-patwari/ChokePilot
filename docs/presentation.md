# ChokePilot — Autonomous Choke Controller
### Honeywell Campus Connect Hackathon (PS3)

Speaker notes are in *italics* under each slide. Numbers below are pulled from
one fresh run of `identify.py` → `verify_identification.py` → `scenarios.py` →
`baselines.py` → `seed_sweep.py` → `scenario_d.py`, cross-checked against
`docs/report.md` — "§N" references point back there for the full derivation.

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
  (0→100% in 10% steps, 40h dwell) against the calibrated simulator, rather
  than reusing the reference CSV.
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
| Q | 5.00 | 4.95 | 3.0% | 0.0–5.0% |
| WHP | 7.50 | 7.20 | 4.0% | 0.0–6.7% |
| FLP | 7.00 | 6.95 | 3.6% | 0.0–7.1% |
| BHP | 9.00 | 8.25 | 8.3% | 2.8–13.9% |

*Dead-time identification now matches the true θ **exactly in all 20/20
seed×channel combinations** — no residual off-by-one anywhere on this table.
BHP is the largest remaining τ residual (8.3%); Slide 4 explains why this
table looks different from an earlier draft.*

---

### Slide 4 — Two Fixes That Explain the Identification Error *(report.md §1.2)*

*Two separate, sequential, tested-not-hypothesized fixes sit behind Slide 3's
table — worth naming both, since stopping after the first would have missed
the second.*

**Fix 1 — dwell time (`DWELL_HOURS` 24→40).** With a 24h dwell, mean τ error
was Q 20% / WHP 16% / FLP 27% / BHP 31%, identified τ lower than true τ in
**all 20/20** seed×channel combinations — a one-directional bias consistent
with insufficient settling time. Raising the dwell to 40h confirmed that
diagnosis for **three of four channels** (WHP 16.0%→4.0%, FLP 27.1%→13.6%,
BHP 31.1%→7.8%) and *ruled it out* for Q (20.0%→19.0%, unchanged across every
dwell length tested, 24–80h) — a genuine open question at that point, not
folded into an explanation that didn't cover it.

**Fix 2 — a one-sample lag between the fitting code and the live simulator.**
`_simulate_fopdt` drove `sim[k+1]` from `y_ss[k]` (i.e. `u[k-θ]`), but
`Simulator.step()` actually drives the arriving sample from `u[t-θ]` — the
same time index, not one behind. A real structural bug, not a hypothesis,
confirmed by tracing both code paths by hand. Fixing it (`y_ss[k]` →
`y_ss[k+1]`) is what **resolved Q's open question**: error dropped
19.0%→3.0%, and θ went from matching exactly for only 2/4 channels to
matching exactly for **all 4**. It also revealed the simulator's own
"ground-truth" θ values had carried the identical bias, so recalibrating
shifted the *true* θ by +1 across the board (τ unchanged) — both sides of
every earlier comparison had been shifted the same way.

*Net effect: BHP is now the largest residual (8.3%), not Q — the direct
consequence of Fix 2 landing squarely on Q's bias.*

---

### Slide 5 — Hybrid Physics + Learned Correction: Three of Four Channels Earn Their Place *(report.md §1.3)*

- A small degree-1 polynomial-in-choke correction on the physics FOPDT
  residual, validated on a **held-out** step test before being trusted.

| Channel | Physics-only RMSE | +Correction RMSE | Kept? |
|---|---|---|---|
| BHP | 9.85 | 9.81 | ✅ **used** |
| FLP | 0.92 | 0.89 | ✅ **used** |
| Q | 1.66 | 1.63 | ✅ **used** |
| WHP | 1.30 | 1.30 | ❌ skipped |

*This table changed shape after Slide 4's Fix 2: with the old, laggy fit,
only BHP's correction generalized to held-out data. Post-fix, Q, FLP, and BHP
all keep a small but consistent held-out improvement; only WHP's correction
makes things very slightly worse on held-out data (1.296→1.304) and is
skipped. `select_beneficial_corrections()` makes this call automatically per
channel from held-out RMSE alone — nothing hand-picked, which is also why the
verdict flipped for three channels without anyone hand-editing it. BHP
remains both the largest τ residual (Slide 3) and the largest correction
RMSE — plausibly absorbing some of that dynamics mismatch; this pipeline
can't distinguish the two.*

---

### Slide 6 — Safety Limits Are One-Sided, Not Symmetric Bands *(report.md §1.4)*

| Channel | Direction enforced | Limit | Brief basis |
|---|---|---|---|
| WHP | floor only (`hi = +inf`) | ≥ 205 psi | *"If WHP becomes too low, the well may operate outside its recommended operating envelope."* High WHP just means the choke is closed back further — safe. |
| BHP | floor only (`hi = +inf`) | ≥ 2830 psi | *"one of the most important indicators of reservoir health and drawdown"* — low BHP means excessive drawdown; high BHP means safely choked back. |
| FLP | ceiling only (`lo = -inf`) | ≤ 200 psi | *"helps ensure stable transportation of produced fluids"* — the risk is backpressure/separator overpressure on the high side, not a low reading. |

*An earlier draft bracketed all three symmetrically, which could reject a
high-WHP/high-BHP or low-FLP state that isn't actually unsafe. These are
further tightened inside the controller (Slide 8) before use.*

---

### Slide 7 — Monitored, Not Constrained: WHT & AP *(report.md §1.5)*

- `data/simulator.py` also simulates **Wellhead Temperature (WHT)** and
  **Annulus Pressure (AP)** — plotted (greyed out) in every scenario figure,
  but they **never feed the controller**.
- They exist because the brief lists them as part of "a complete production
  operating envelope" an operator would want visibility into — not because
  this challenge's control problem is defined over them.
- WHT declines monotonically with choke opening (Joule-Thomson cooling); AP
  is flat, decoupled from choke position, carrying only an illustrative
  integrity-alarm band (1650–1950 psi) for situational awareness.
- Neither has a reference-CSV column to calibrate against, so unlike
  Q/WHP/FLP/BHP their curve parameters are hand-set placeholders — chosen to
  be qualitatively right, not fit to data.

*Deliberate design line: surfaced for situational awareness — echoing
Honeywell Asset Sentinel's risk/alert layer — but a WHT/AP reading can never
change which choke move the controller picks.*

---

## Section 2 — Control Strategy

---

### Slide 8 — What This Actually Is: One-Step Receding-Horizon Search, Not Full MPC *(report.md §2.1, §2.2)*

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
   closest to target, ranked with a move-suppression penalty (Slide 14) so
   near-ties don't get decided by noise; if none is feasible, fall back to
   the one minimizing predicted violation, same penalty applied — the
   controller never freezes or refuses to act.

- This **is** receding-horizon in the classic sense — it re-runs every
  interval with fresh measurements — but the per-step search itself is a
  one-step decision screened by a hold-constant forward simulation. Horizon
  is derived, not hand-tuned: `ceil(3 × max(τ) / Ts)`, clipped to [3, 12] h.
- **Safety here is best-effort, not formally guaranteed** — no recursive
  feasibility check, no terminal invariant set. The 30-seed sweep (Slide 15)
  shows the safety-fallback branch firing on **23.6%** of Scenario C's steps
  — precisely because that scenario runs at the edge of what the model
  considers feasible, with no formal guarantee behind it, only a greedy
  per-step search that has worked well empirically.

---

### Slide 9 — Scenario A's Violations: A Deterministic Initial-Condition Property, Not an Extrapolation Artifact *(report.md §2.3)*

*Two earlier drafts got this wrong: the first claimed 15% choke was "inside
supported territory" (false — the CSV's tested band is 30–65%); the second
correctly retracted that but blamed the violations on generic "extrapolation
uncertainty," implying the numbers might not be real. Checking the model's
own numbers directly says otherwise.*

Computing the identified FLP steady-state map at low choke (the exact
function the controller predicts with):

| Choke | FLP steady-state | vs. 200 psi ceiling |
|---|---|---|
| 0% | 218.4 psi | exceeds |
| 15% | 203.7 psi | exceeds |
| 20% | 198.8 psi | clears (barely) |
| 25%+ | 193.9 psi | clears with margin |

**Scenario A starts at 15% choke, and `Simulator.reset()` initializes FLP at
exactly that choke's steady state — 203.7 psi, already over the ceiling,
before the controller has made a single decision.** With the ±5%/interval
ramp limit, reaching even the steady-state-safe 20% choke takes one full
interval, and reaching the controller's own tightened-margin-safe ~22% takes
two — a hard constraint the controller has no authority to override, plus
FLP's own τ=7h/θ=1h lag on top.

*This is the identified model's own curve, trusted exactly as identified —
not an artifact of guessed, untrustworthy numbers outside the calibrated
range. A better 0–30% calibration could shift the exact numbers; it would not
change the qualitative outcome unless it also flipped the sign of FLP's slope
near 15%, which nothing in the physics suggests.*

---

### Slide 10 — Explainability Is a Logged Field, Not a Slide *(report.md §2.4)*

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

*Single-run numbers below use the default seed; the 30-seed sweep (Slide 15)
is the number to trust over any one run.*

---

### Slide 11 — Scenario A: Startup to Target *(report.md §3.1)*

- 15% choke → 100 bbl/hr target, 80-hour run.
- **Tracking:** trailing 10-hour mean (hours 70–79) of **99.26 bbl/hr** at a
  steady 34.0% choke.
- **Safety, single seed:** 4/80. **30-seed sweep:** every seed shows at
  least one violation (**30/30**), mean **3.97/80**, max **5/80**.
- **Time to enter the safe envelope** (new metric, Slide 9's mechanism made
  measurable): single-seed **4h**; 30-seed sweep **mean 4.0h, range 3–5h** —
  tight enough to support "deterministic," not "noisy extrapolation."
- **Ramp-rate:** 0/80 violations in every seed — the ramp limit is the
  *reason* the envelope takes several hours to reach, not itself violated.

---

### Slide 12 — Scenario B: Target Tracking *(report.md §3.2)*

- 34.2% choke start, target steps **100 → 150 bbl/hr at t = 60h**, 140-hour
  run.
- **Tracking:** trailing 10-hour mean **100.32 bbl/hr** at 34.2% choke
  before the step (hours 50–59); **150.57 bbl/hr** at 61.2% choke after
  settling (hours 130–139).
- **Safety:** 0/140 in the representative run **and** 0/140 in **all 30
  seeds** — the only scenario with a perfectly clean sweep, because it never
  leaves the calibration's supported 30–65% band.

---

### Slide 13 — Scenario C: Infeasible Target *(report.md §3.3)*

- 34.2% choke start, **400 bbl/hr requested** — deliberately beyond what's
  safely achievable.
- **Behavior:** does not chase the target into a violation. Trailing
  10-hour mean **162.76 bbl/hr** at 69.4% choke — the maximum rate the
  tightened envelope allows, not 400.
- **Safety:** 0/100 single-seed, **and 0/100 in all 30 seeds** — fully
  clean.
- **Safety-fallback frequency:** **23.6%** of steps — far higher than A or
  B, because this scenario deliberately runs the choke near the edge of the
  tightened feasible envelope for its entire duration. Frequent fallback
  here is expected behavior, not a red flag: 0 actual violations means the
  fallback branch is doing its job.

---

### Slide 14 — Actuator Activity: Fixing the Chattering *(report.md §3.4)*

*The controller originally picked the feasible candidate purely by predicted
target error, with move size only a last-resort tie-break. Near target, many
candidates' errors differ by less than measurement noise — the "closest" one
was effectively a coin flip, so the valve chattered chasing error it can't
actually predict.*

**Fix:** move-suppression cost on both branches — `cost = q_err + λ·|Δu|`
(feasible) / `cost = violation + λ·|Δu|` (fallback), `λ = 1.0` bbl/hr of
predicted improvement required per %-point moved.

| Scenario | Moves | Total valve travel |
|---|---|---|
| A | 5 / 80 | 16.0 %-pts |
| B | 7 / 140 | 29.0 %-pts |
| C | 53 / 100 | 129.0 %-pts |

*Honest result, not fully positive: the fix works as intended for A and B —
5 and 7 moves total, vs. constant hunting beforehand. **Scenario C is only
partially improved**: travel dropped 16% (154→129 %-pts) but move count is
essentially unchanged (52→53). Root cause: C rides the tightened WHP floor
with real measurement noise (σ≈1.2 psi) that flips **which candidates are
feasible at all**, step to step — a ranking fix within the feasible set
can't fix the feasible set itself changing.*

---

### Slide 15 — Seed-Sweep Distribution: 30 Seeds per Scenario *(report.md §3.5)*

| Scenario | Mean violations | Max violations | Seeds with ≥1 violation | Mean safety-fallback rate |
|---|---|---|---|---|
| A — Startup to Target | 3.97 / 80 | 5 | 30 / 30 | 8.58% |
| B — Target Tracking | 0.00 / 140 | 0 | 0 / 30 | 0.00% |
| C — Infeasible Target | 0.00 / 100 | 0 | 0 / 30 | 23.57% |

*Full per-seed results: `outputs/seed_sweep_results.csv`. A single seed's "0
violations" is a point sample from a noisy system — this distribution, not
any one run, is what actually supports a safety claim.*

---

### Slide 16 — Baseline Comparison: MPC vs. Fixed-Optimal vs. Fixed-Operator-Proxy vs. PI *(report.md §3.6)*

![Baseline comparison: constraint violations (log scale) and total barrels produced, per approach, per scenario](../outputs/baseline_comparison.png)

- **Fixed-optimal** — the choke the identified model says holds the target
  at steady state, walked back until its own steady-state predictions clear
  the tightened envelope, then held. Model-informed, envelope-aware — what a
  good engineer *with* this pipeline would set.
- **Fixed-operator-proxy** (new) — **no model, no envelope knowledge**: a
  naive straight-line read of choke-vs-oil-rate off the raw reference CSV
  (the only data a real operator without this pipeline would have), backed
  off 15 points. Models pain point #1 directly.
- **PI** — velocity-form PI on oil rate, IMC-tuned from the identified
  model, blind to WHP/FLP/BHP by design.

| Scenario | Approach | Safety violations | Total barrels |
|---|---|---|---|
| A | MPC | 4/80 | 7,590.4 |
| A | Fixed-optimal | 3/80 | 7,657.6 |
| A | Fixed-operator-proxy | **66/80** | 4,852.4 |
| A | PI | 3/80 | 7,652.1 |
| B | MPC | 0/140 | 17,676.4 |
| B | Fixed-optimal | 0/140 | 17,642.0 |
| B | Fixed-operator-proxy | **23/140** | 13,824.5 |
| B | PI | 0/140 | 17,591.0 |
| C | MPC | 0/100 | 15,806.2 |
| C | Fixed-optimal | 0/100 | 15,769.0 |
| C | Fixed-operator-proxy | **85/100** | 18,778.3 |
| C | PI | **85/100** | 18,778.3 |

*The honest headline — not the one Scenario D (Slide 17) was built to find:
**MPC ties Fixed-optimal in every scenario** (no cross-channel coupling for
live re-planning to exploit here). The real, consistent win is
**envelope-awareness itself vs. the operator-proxy**, in all four scenarios —
closing the choke 15% below naive belief looks conservative but actually
pushes FLP the *wrong* direction, past its ~19% threshold (Slide 9), which a
model-blind operator has no way to know.*

---

### Slide 17 — Scenario D: Disturbance Rejection *(report.md §3.6)*

*Built specifically to find MPC's edge: does live re-planning beat a static
setpoint once the plant genuinely drifts underneath both? A deliberate,
explicit relaxation of the brief's "no changing reservoir properties"
simplification.*

- BHP's identified steady-state offset drifts **−0.5 psi/h for 200h**
  (reservoir decline) at a constant 100 bbl/hr target. The identified model
  is fit once, before the disturbance starts, and never updated.

| Approach | Safety violations | Total barrels | Time-to-first-violation |
|---|---|---|---|
| MPC | 0/200 | 20,040.9 | never |
| Fixed-optimal | 0/200 | 20,048.3 | never |
| Fixed-operator-proxy | **121/200** | 12,594.2 | **21h** |

**Told straight, not spun:** MPC and Fixed-optimal **tie** — 0 violations
each. In this model, BHP and Q are independent, uncoupled FOPDT channels, so
a declining reservoir never moves Q off target and never drifts BHP close
enough to its floor (ends ~170 psi above it after 200h) to matter. MPC's
re-planning had nothing to correct that a static, envelope-aware setpoint
didn't already handle. Fixed-operator-proxy still fails here — but for the
*same static-FLP-threshold reason as Scenario A/Slide 9*, not because it
missed the disturbance: violations start at hour 21 (as soon as the
ramp-limited approach reaches its 18.7% setpoint) and continue at a roughly
steady rate for all 200h.

---

### Slide 18 — Lessons Learned *(report.md §3.7)*

- **"Extrapolation" was a hedge, not the actual explanation.** Checking the
  model's own FLP steady-state curve (Slide 9) found a concrete, checkable
  fact — it exceeds 200 psi below ~19% choke — not a hedge about what an
  uncalibrated region *might* do.
- **A correctly-specified model still shows real identification error, and
  two separate root causes were both worth chasing down, not just naming
  one.** Dwell time (Slide 4, Fix 1) explained three of four channels and
  correctly *ruled itself out* for Q; a one-sample lag (Fix 2) was Q's actual
  answer, found by refusing to let Fix 1's partial success explain away the
  residual it didn't touch.
- **Name the controller accurately.** Calling this "brute-force MPC" implied
  guarantees — recursive feasibility, trajectory optimality — it doesn't
  have. It's a one-step receding-horizon search with hold-constant
  prediction: empirically strong on safety, but not formally proven.
- **Report distributions, not single runs.** A single seed's "0 violations"
  is a point sample. The 30-seed sweep is what actually supports a safety
  claim or a tracking claim.
- **A correction layer earns its place by generalizing, channel by
  channel.** Post-fix, it helps three of four channels (Q, FLP, BHP) and is
  skipped only for WHP — the verdict changed entirely from an earlier draft
  without anyone hand-editing which channels are marked used.
- **Chattering has more than one cause, and a single fix doesn't
  necessarily cure both.** Move-suppression cost fully fixed A/B (noise-tie
  chattering) but only partially fixed C, because C's chattering comes from
  the *feasible set itself* flipping under measurement noise — a different
  mechanism the same fix can't fully reach.
- **The real win is envelope-awareness, not live re-planning — say so even
  though it complicates the headline.** Scenario D was built to find MPC's
  edge over a static setpoint and didn't find one (they tie). The
  consistent, honest win is against the operator-proxy baseline, in all
  four scenarios — a less dramatic but more accurate story than the one the
  experiment was designed to produce.
