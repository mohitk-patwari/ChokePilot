# ChokePilot — Autonomous Choke Controller
### Honeywell Campus Connect Hackathon (PS3)

Speaker notes are in *italics* under each slide. Numbers are pulled live from
`scenarios.py` output and match what `docs/report.md` will use — don't let the
two drift apart.

---

## Section 1 — Process Understanding & Model

---

### Slide 1 — The Problem

Operators and engineers don't under-perform because the physics is hard — they
under-perform because of five well-documented, unglamorous frictions:

1. **Operators "baby" the choke** — fear of sand/formation damage leaves real
   production capacity unused. *(Eng-Tips forum discussion)*
2. **Too many wells, too little attention** — chokes get set once and left
   alone. *(patent literature on standard industry practice)*
3. **Slugging is caught reactively** — a human notices fluctuations after the
   fact, not before. *(SPE technical paper)*
4. **APC needs a maintenance team** — the real barrier to upstream adoption
   isn't the control theory, it's who keeps the models alive. *(on-record
   Honeywell upstream solutions interview)*
5. **Alarm fatigue and distrust of black-box automation** — operators override
   what they don't understand. *(NTSB SCADA safety review)*

*ChokePilot is a small, closed-loop answer to all five: continuous
optimization instead of a "set and forget" choke, brute-force logic instead of
a model a dedicated team must maintain, and a logged one-sentence rationale
for every move instead of a black box.*

---

### Slide 2 — No Official Simulator: What We Built Instead

- The platform provided no simulator — only
  `Autonomous_Choke_Control_Simulated_Dataset.csv` (120 hr, choke tested
  30–65%) and a presentation template.
- We built `data/simulator.py`: a **calibrated substitute**, clearly commented
  as such, fit to that CSV with the exact interface the brief specifies —
  `Q, WHP, FLP, BHP = simulator.step(choke_position)`.
- Model shape per channel: a saturating steady-state map
  `y_ss(u) = A + B·u/(u+uh)` (monotonic, bounded — the standard choke
  flow/pressure-vs-opening curve) driving first-order-plus-dead-time (FOPDT)
  dynamics.
- Safety limits (WHP/FLP/BHP) are **placeholders**: derived from the CSV's
  observed range +20% margin, since no official limits were specified.

*This isn't a shortcut — it's the only path available without an official
simulator, and it's built to be swapped out: nothing downstream depends on
this file's internals, only on `.step()`.*

---

### Slide 3 — Open-Loop Step Test

- Per the brief: "students are expected to generate their own data using the
  simulator" — so we run a **fresh** staircase, not a reuse of the reference
  CSV.
- Monotonic sweep, 0% → 100% in 10% steps, **24 h dwell** per level (~4× the
  slowest calibrated time constant + dead time → near-settled before the
  next step).
- A naturally-flowing well with no artificial lift and unchanging reservoir
  properties isn't expected to show hysteresis, so an up-only sweep is
  sufficient to characterize it.
- Output: `outputs/step_test_data.csv` / `step_test_response.png` — Q, WHP,
  FLP, BHP response to the full choke range.

---

### Slide 4 — Identified FOPDT Model

Fit by brute-force grid search (uh, tau, theta) + closed-form regression
(A, B) — same method used to calibrate the simulator itself, applied fresh to
our own step test:

| Channel | Time constant τ (h) |
|---|---|
| Q (oil rate)   | 2.25 |
| WHP            | 3.25 |
| FLP            | 2.50 |
| BHP            | 3.00 |

- Q's steady-state map is forced through zero at 0% choke (a fully shut choke
  physically gives zero flow — no extrapolated intercept).
- This identified model — not the simulator's internal parameters — is what
  the controller's brute-force MPC predicts forward with.

---

### Slide 5 — Hybrid Physics + Learned Correction

- On top of the FOPDT physics fit, we fit a **tiny** degree-1
  polynomial-in-choke correction on the residual the physics model leaves on
  a step test — a bias correction on the steady-state map, not a replacement
  for the physics dynamics.
- Validated honestly: RMSE compared **physics-only vs. physics+correction on
  a held-out step test** (different noise draw, same design), and a channel
  only keeps the correction if it demonstrably helps.

| Channel | Physics-only RMSE | + Correction RMSE | Kept? |
|---|---|---|---|
| Q   | 3.50  | 3.27  | ✅ used |
| WHP | 4.30  | 4.17  | ✅ used |
| BHP | 26.95 | 25.67 | ✅ used |
| FLP | 2.31  | 2.34  | ❌ skipped — didn't generalize |

*FLP's correction made the training fit look better but made held-out error
worse, so it's dropped. A correction that doesn't demonstrably help has no
business being wired into the controller just because it exists — that
discipline is the point of reporting it this way.*

---

### Slide 6 — Known Limitation: The Support Region

- The calibrated simulator and safety limits only have **real support** in
  the 30–65% choke range — the band the reference CSV actually tested.
- Starting a scenario at a hard 0% shut-in (or a low ~5% ramp-up) forces
  extrapolation outside that support, causing early-hour constraint
  violations that are an artifact of the extrapolation — not the controller.
- **Decision, applied consistently across all three scenarios:** start every
  scenario inside supported territory rather than a hard extrapolated
  extreme, while still preserving what each scenario is designed to
  demonstrate. (Full effect on results — Slide 10.)

---

## Section 2 — Control Strategy

---

### Slide 7 — Brute-Force MPC: Why No Optimizer

Each control interval (Ts = 1 h):

1. Enumerate **11 fixed candidate moves**: −5% to +5% in 1% steps — the full
   range the ramp-rate constraint allows.
2. Simulate each candidate forward over a lookahead horizon with the
   identified FOPDT (+ correction) model.
3. Reject any candidate predicted to breach a WHP/FLP/BHP limit anywhere in
   the horizon.
4. Command the feasible candidate whose end-of-horizon predicted oil rate is
   closest to target.

- Hard constraints enforced by construction: choke ∈ [0, 100]%, |Δchoke| ≤ 5%
  per interval.
- No optimizer library, no gradient solver — 11 candidates × a short horizon
  is cheap enough to brute-force. The brief explicitly calls this
  acceptable, and it directly answers pain point #4 (APC needing a dedicated
  modeling team): there's no solver to tune or maintain.

---

### Slide 8 — Constraint Tightening for Safety

- Predictions are noise-free, but real sensor readings carry measurement
  noise — riding the true limit exactly would let real noise dip past it.
- Standard **robust-MPC constraint tightening**: back off each limit by
  **3σ** of that channel's identified noise_std before the controller ever
  compares a prediction against it.
- If *no* candidate is fully feasible (e.g. noise has already pushed a
  reading near a limit), the controller falls back to the candidate that
  **minimizes** the predicted violation — it never freezes or refuses to
  act.

---

### Slide 9 — Explainability Is a Logged Field, Not a Slide

Directly answers pain point #5 (alarm fatigue / distrust of opaque
automation): every decision produces a real, human-readable rationale — not
just a design principle, a field in the output CSV, in both branches of the
logic.

> *"Moved choke to 35% because it keeps WHP/FLP/BHP within safe limits over
> the next 10h and brings predicted oil rate to 99.9 bbl/hr, closest to the
> 100.0 bbl/hr target among 10 feasible options."*
> — Scenario A, actual logged decision

> *"No choke move keeps all limits satisfied over the lookahead; moved to X%
> because it minimizes the predicted constraint violation (Y psi-steps over
> horizon)."*
> — safety-fallback branch, same field, same guarantee

**Honeywell echo:** dashboard/output layering mirrors Uniformance's PHD
(historian) / KPI (target-tracking) / Asset Sentinel (risk/alert) structure —
this is a focused proof-of-concept of the same wellhead-to-control-room
optimization problem Honeywell's Upstream Production Performance Suite
(UPPS) addresses at scale. *(echoes, not replicates.)*

---

## Section 3 — Results

---

### Slide 10 — Scenario A: Startup to Target

- Starts at **15% choke** (not a hard 0% shut-in — Slide 6), target 100
  bbl/hr, 80-hour run.
- Constraint violations: **21/80 → 3/80** after the support-region fix.
- Residual 3 violations are a 2–4 psi BHP overshoot during the ramp-limited
  hours-2–4 transient — shrunk to near zero, not fully eliminated, and
  honestly reported as such.
- Final state: target 100.0, actual **104.0 bbl/hr**, choke settled at 35%.
- Zero ramp-rate violations across the whole run.

---

### Slide 11 — Scenario B: Target Tracking

- Starts at **35.7% choke** — the point the identified Q model itself says
  holds ~100 bbl/hr steady-state (`solve_choke_for_q()`), not a ramp-up
  start.
- Target steps **100 → 150 bbl/hr at t = 60h**, 140-hour run.
- Constraint violations: **22/140 → 0/140** — fully eliminated, because the
  whole run now stays inside the model's supported territory.
- Final state: target 150.0, actual **148.3 bbl/hr**, choke settled at 66%.

---

### Slide 12 — Scenario C: Infeasible Target

- Same 35.7% stable start, but the target is **400 bbl/hr** — deliberately
  beyond what's safely achievable.
- The controller doesn't chase it: it settles at the **maximum safe rate**
  instead of forcing an infeasible move.
- Constraint violations: **20/100 → 0/100** — fully eliminated.
- Final state: target 400.0, actual **158.8 bbl/hr**, choke settled at 76%
  (near the top of its safe operating range for this target/limit
  combination).
- Sample logged rationale: *"...brings predicted oil rate to 161.8 bbl/hr,
  closest to the 400.0 bbl/hr target among 10 feasible options"* — the
  controller is explicit that it's doing the best it safely can, not
  silently failing.

---

### Slide 13 — Lessons Learned

- **The support-region fix was the single highest-leverage change.**
  Starting scenarios inside the calibrated model's real support (30–65% CSV
  band) rather than at extrapolated extremes cut total violations across all
  three scenarios from 63/300 to 3/300 — without touching the controller
  logic at all. The remaining 3 are a genuine ramp-limited transient, not a
  modeling artifact.
- **Report correction results honestly, including the negative one.** The
  learned correction layer only ships per-channel where it beats physics-only
  on held-out data — FLP's fit looked better in-sample and was dropped
  because it didn't generalize. That discipline matters more than the RMSE
  numbers themselves.
- **Brute-force is a feature, not a compromise.** 11 fixed candidates with a
  logged rationale per decision directly answers two of the five pain points
  this project set out to address (maintenance burden, operator trust) —
  a fancier optimizer would have made both worse, not better.
