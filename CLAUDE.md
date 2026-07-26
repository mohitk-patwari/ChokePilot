# ChokePilot — Autonomous Choke Controller (Honeywell PS3)

## Hard constraints (never violate)
- Choke opening: 0-100%. Max change per control interval: +/-5%.
- Control interval (Ts): 1 hour.
- WHP, FLP, BHP must stay within safe limits at all times — reject any candidate
  action that would violate them.
- Simulator interface: Q, WHP, FLP, BHP = simulator.step(choke_position)

## Status: no official simulator was provided by the platform
Only data/Autonomous_Choke_Control_Simulated_Dataset.csv (reference-only, 120hr,
choke tested 30-65%) and a presentation template exist. data/simulator.py is a
CALIBRATED SUBSTITUTE fit to that CSV (saturating steady-state map + FOPDT dynamics),
clearly commented as such. Safety limits are PLACEHOLDERS derived from the CSV's
observed range +20% margin, then further tightened by 3-sigma of each channel's
identified measurement noise (robust-MPC constraint tightening) so real sensor
noise doesn't breach the true limit even when a noise-free prediction rides it.
Limits are one-sided, not symmetric bands: WHP and BHP are lower-bounded only
(hi=inf -- high pressure means safely choked back; the risk is too LOW, per the
brief's "if WHP becomes too low..." line and BHP's drawdown/reservoir-health role),
FLP is upper-bounded only (lo=-inf -- backpressure/separator risk is on the high
side). See safety_limits_from_reference() in controller.py for the per-channel
brief-quote justification.

## Known limitation (deliberately handled, not a bug)
The calibrated simulator/limits only have real support in the reference CSV's
tested 30-65% choke range (only 5 distinct choke levels were ever observed, all
>=30%). Starting any scenario at a hard 0% shut-in (or a low ~5% ramp-up) forces
the model deep into unfitted territory.

CORRECTED (was wrong twice): the violations this causes are NOT an
"extrapolation uncertainty" artifact -- that framing implies the numbers might
not be real. They're a DETERMINISTIC INITIAL-CONDITION property, confirmed by
computing the identified model's own FLP steady-state curve directly:
FLP_ss(u) exceeds the 200 psi ceiling for any u below ~19% choke (218.4 psi at
0%, 208.6 at 10%, 203.7 at 15%, clears at 20%). Matching docs/report.md
Sections 2.3/3.1 exactly; do not restate a softer "extrapolation" version of
this elsewhere:
- Scenario A starts from 15% choke instead of 0%. `Simulator.reset()`
  initializes FLP at exactly that choke's steady state -- 203.7 psi, already
  over the true ceiling, before the controller acts at all. With the
  +-5%/interval ramp limit, reaching the steady-state-safe 20% takes 1 step,
  and reaching the controller's own tightened margin (~197.3 psi, crossed
  ~21-22% choke) takes 2 -- the controller has no authority to move faster
  than that regardless of how it weighs the tradeoff. FLP's own dynamics
  (tau=7h, theta=1h) add further lag on top.
  NEW METRIC -- time to enter the safe envelope (hour after which no further
  violations occur): single-seed 4h; across the 30-seed sweep, mean 4.0h,
  range 3-5h. This tightness is itself evidence for "deterministic," not
  "extrapolation noise" -- an uncertainty-driven artifact would show more
  seed-to-seed spread.
  Violation count, from the 30-seed sweep (seed_sweep.py) -- the number to
  trust, not a single cherry-picked run: every seed shows at least one
  violation (30/30), mean 3.97/80 steps, max 5/80. Unaffected by the
  DWELL_HOURS=40 fix below (identification accuracy doesn't change what the
  identified FLP curve says at low choke, which is the actual cause).
- Scenarios B and C don't need a startup transient at all (that's Scenario A's
  job), so they start at the choke the identified Q model itself says holds
  ~100 bbl/hr steady-state (solve_choke_for_q() in scenarios.py, ~34.2%) --
  fully inside the calibrated 30-65% band, not just closer to it.
  Result across the 30-seed sweep: 0/140 violations in every seed (B), 0/100
  violations in every seed (C) -- both fully clean, because both scenarios
  stay in supported territory for their entire run.

Extrapolation uncertainty is a property of the substitute simulator's
calibration -- not something the controller can detect or correct for. The
controller has no way to know its own prediction model is running on
unvalidated territory; it simply evaluates the fitted curve wherever it's
asked to. This is a deliberate, defensible modeling choice, not a shortcut.

## Principles
- Think before coding. State assumptions before implementing.
- Prefer the simplest thing that works. Brute-force MPC candidate evaluation is
  explicitly acceptable — do not reach for heavy optimization libraries by default.
- Surgical changes only — don't refactor unrelated code while fixing one thing.
- Every controller decision must be explainable in one sentence (why this choke
  position, given current state) — this must be a real logged field, not just
  a principle; verify it actually exists in controller.py's output.
- Commit to git after every completed stage. Never leave uncommitted working code.

## Project structure (as actually built)
- data/simulator.py — calibrated substitute simulator (see Status above)
- data/Autonomous_Choke_Control_Simulated_Dataset.csv — reference-only, do not
  reuse directly for model ID (per brief: generate fresh data instead)
- identify.py — fresh open-loop step test + FOPDT model identification
- controller.py — brute-force MPC controller (11 candidates, constraint tightening)
- scenarios.py — runs Scenarios A/B/C, saves required plots to outputs/
- outputs/ — generated plots (target vs actual oil rate, WHP, FLP, BHP, choke)
- docs/ — architecture doc, report (not yet written)

## Scenario definitions (exact, from problem statement)
- Scenario A - Startup to Target: well starts at 15% choke (see Known Limitation
  above), controller brings it to a 100 bbl/hr target.
- Scenario B - Target Tracking: starts at the choke holding ~100 bbl/hr steady-state
  (~34.2%, see Known Limitation above), target steps 100 -> 150 bbl/hr partway
  through. Must respect WHP/FLP/BHP and ramp-rate constraints throughout.
- Scenario C - Infeasible Target: starts at the same ~34.2% stable point, 400 bbl/hr
  target exceeds what's safely achievable. Controller must reject it and settle at
  max achievable safe rate.

## Pain points this project addresses (use this framing in docs/comments, not
just as background — this is meant to justify design decisions)
1. Operators "baby" the choke conservatively (fear of sand/formation damage),
   leaving real production capacity unused — source: Eng-Tips forum discussion.
2. Production engineers manage too many wells to give each one continuous
   attention — chokes get set once and left alone — source: patent literature
   describing standard industry practice.
3. Slugging/instability is typically caught only after a human notices
   fluctuations (reactive, not preventive) — source: SPE technical paper.
4. Real barrier to APC adoption upstream is needing a dedicated team to maintain
   the models — source: on-record Honeywell upstream solutions interview. Our
   brute-force (no heavy optimization library) approach directly answers this.
5. Alarm/controller fatigue and operator distrust of opaque automated decisions
   — source: NTSB SCADA safety review. Answered by the one-sentence rationale
   logged per controller decision.

## Honeywell product-awareness framing (for architecture doc / presentation —
do NOT overclaim equivalence, frame as "echoes"/"mirrors", never "is" or "replicates")
- Honeywell APC has customers using closed-loop control on wells, moving upstream
  toward autonomy — this project is a small-scale version of that same direction.
- Dashboard/output structure should echo Honeywell Uniformance's PHD (historian)
  / KPI (target-tracking) / Asset Sentinel (risk/alert) layering.
- One sentence: this is a focused proof-of-concept of the same wellhead-to-
  control-room optimization problem Honeywell's own Upstream Production
  Performance Suite (UPPS) addresses at scale.

## Required deliverables
Python notebook/code, open-loop step-test analysis, dynamic model ID, controller
implementation, results for Scenarios A/B/C with plots (target vs actual oil rate,
WHP, FLP, BHP, choke position), architecture/report doc, presentation slides.

## Current build status (update this section as work progresses)

DONE:
- Core pipeline: simulator.py (calibrated substitute -- steady-state fit on only
  the last ~6h of each dwell period, widened uh grid with explicit
  boundary-pinning report when uh is unidentifiable, exact ZOH discretization
  `alpha = 1 - exp(-Ts/tau)` in place of the Euler approximation), identify.py
  (fresh step test + FOPDT identification + hybrid physics+learned-correction
  layer), controller.py (one-step receding-horizon search: 11 fixed candidates,
  hold-constant-over-horizon prediction, constraint-tightening margin --
  NOT full trajectory-optimizing MPC and NOT a formal safety guarantee; see
  docs/report.md Section 2.1), scenarios.py (A/B/C runs + plots).
- Constraint-direction fix: WHP/BHP lower-bounded only, FLP upper-bounded only
  (a symmetric band was rejecting non-hazardous high-WHP/high-BHP/low-FLP
  states) -- see Status above.
- Scenario A/B/C startup-condition fix and its honest, corrected framing --
  see Known Limitation above (15% reduces but does not eliminate Scenario A's
  deterministic FLP-ceiling violations; B/C start fully inside the calibrated
  band).
- WHT (wellhead temperature) and AP (annulus pressure) added to
  data/simulator.py as monitored-but-not-constrained channels -- hand-set
  placeholder params (no CSV column exists for either), never feed the
  controller's safety check or MPC objective, plotted greyed-out in
  scenarios.py, read via Simulator.read_monitored(). Documented in
  docs/report.md §1.5, docs/presentation.md, and README.md.
- requirements.txt (pinned numpy/pandas/matplotlib/pytest) and tests/ (41
  pytest tests: ramp-rate/choke-bound actuator constraints, identified-tau
  thresholds across 5 seeds, infeasible-target safety across the 30-seed
  sweep) -- run as the safety gate before every push.
- verify_identification.py fixed (KeyError from WHT/AP being included in
  simulator.PARAMS but never identified by identify.py -- restricted to
  identify.CHANNELS).

**Tier-0 fix -- a one-sample lag between the fitting code and the live
simulator (the single biggest identification-accuracy fix in the project).**
`_simulate_fopdt` drove `sim[k+1]` from `y_ss[k]` (i.e. `u[k-theta]`), but
`Simulator.step()` actually drives the arriving state from `u[t-theta]` --
the same time index as the arrival sample, not one behind it. Fixed by
changing `y_ss[k]` to `y_ss[k+1]` in `_simulate_fopdt` (and the identical bug,
plus a missed Euler-vs-ZOH inconsistency, in identify.py's
`_simulate_with_correction`). This was Q's previously "unexplained" ~19% tau
bias -- theta now matches true theta exactly in 20/20 seed x channel
combinations, Q's tau error dropped to ~3%. Recalibrating simulator.py's own
PARAMS with the fix also revealed the "true" theta values had carried the
identical bias the whole time (now 5.00/1, 7.50/2, 7.00/1, 9.00/3 -- tau
unchanged, theta all +1). Net effect on the identification table: **BHP is
now the largest residual (8.3%), not Q** -- current means/errors, 5 seeds
(0,1,2,7,99): Q 4.95h/3.0%, WHP 7.20h/4.0%, FLP 6.95h/3.6%, BHP 8.25h/8.3%,
all at theta_match_rate=1.0. This fix landed *after* the DWELL_HOURS fix
below and resolved the one channel that fix couldn't touch -- two separate
root causes, not one.

DWELL_HOURS 24->40 fix, confirmed (not just hypothesized) as the cause of
most tau identification bias: WHP/FLP/BHP mean tau error dropped
16%/27%/31% -> 4%/14%/8%. Q's ~19-20% bias was unaffected across every dwell
tested (24-80h) at the time -- correctly reported as a separate open question,
which the Tier-0 fix above later answered.

**Correction layer, re-evaluated post-Tier-0-fix: 3 of 4 channels now used,
not just BHP.** `select_beneficial_corrections()` picks per channel from
held-out RMSE alone, no manual override. Current verdict: Q, FLP, and BHP all
keep a small but consistent held-out improvement (used); WHP's correction
makes held-out RMSE very slightly worse (1.296->1.304) and is skipped. This
verdict flipped for three channels solely because Tier-0 changed the
underlying residuals -- nobody hand-edited which channels are marked used.
BHP remains the channel with both the largest tau error and the largest
correction RMSE.

**Move-suppression (chattering) fix.** MPCController picked candidates by
q_err alone (tie-break on move size only), so once near target, candidates
within noise of each other on q_err got picked by noise -- chattered the
valve (Scenario C: 52 moves / 154 %-points travel over 100h). Added
cost = q_err + LAMBDA_MOVE*abs(delta) (also applied to the fallback branch's
violation-based cost, since Scenario C spends ~24% of steps there).
LAMBDA_MOVE=1.0 (bbl/hr required per %-point moved). Result: Scenario A/B
chattering resolved (5/80 moves / 16.0 %-pts travel; 7/140 moves / 29.0 %-pts
travel). Scenario C only partially improved (travel 154->129, move count
52->53 essentially unchanged) -- root cause is different: C rides the
tightened WHP floor (208.6 psi) with WHP noise (sigma~1.2 psi, observed
swinging 207-213) large enough to flip which candidates are feasible step to
step, which a cost-ranking fix within whichever set is feasible can't solve.
A larger noise_margin_sigma would likely help; not tried.

**Scenario D - Disturbance Rejection (scenario_d.py) + second Fixed baseline,
Fixed-operator-proxy (baselines.py).** BHP's identified steady-state offset
drifts -0.5 psi/h for 200h (reservoir decline, deliberate relaxation of the
brief's "no changing reservoir properties" simplification) at a constant
100 bbl/hr target. Simulator gained a params= constructor arg (defaults to
the shared global PARAMS; pass a private copy.deepcopy to drift one
instance's plant without touching any other scenario). Fixed-operator-proxy
models pain point #1 directly: no model, no envelope knowledge, a naive
linear read of choke-vs-oil-rate off the raw reference CSV, backed off 15
points. The original Fixed baseline was renamed Fixed-optimal for clarity
(still model-informed and envelope-aware, just set once).
CORRECTED FINDING (checked against real data before writing, not assumed):
Scenario D was built to show MPC beating a static setpoint via live
re-planning. It doesn't -- MPC ties Fixed-optimal in ALL FOUR scenarios
(A/B/C/D: 0/200 vs 0/200 violations, 20040.9 vs 20048.3 barrels in D), because
BHP and Q are independent, uncoupled FOPDT channels in this model, so the
reservoir decline never gives MPC's re-planning anything to correct that
Fixed-optimal's envelope-aware static setpoint didn't already handle. The
real, consistent, four-for-four win is MPC/Fixed-optimal (both
envelope-aware) vs. Fixed-operator-proxy (not) -- e.g. D: 0/200 vs 121/200
violations, 20048.3 vs 12594.2 barrels, first violation at 21h. Mechanism:
backing off 15% closes the choke, which is safe for WHP/BHP (lower-bounded)
but UNSAFE for FLP (upper-bounded) -- the same static ~19%-choke FLP-ceiling
mechanism as Scenario A (see Known Limitation above), confirmed directly in D
(violations start at hour 21, steady rate for all 200h -- the disturbance is
a minor factor at most, not the cause). Current baseline-comparison numbers
(all four scenarios, all four approaches): A -- MPC 4/80/7590.4,
Fixed-optimal 3/80/7657.6, Fixed-operator-proxy 66/80/4852.4, PI 3/80/7652.1;
B -- MPC 0/140/17676.4, Fixed-optimal 0/140/17642.0, Fixed-operator-proxy
23/140/13824.5, PI 0/140/17591.0; C -- MPC 0/100/15806.2, Fixed-optimal
0/100/15769.0, Fixed-operator-proxy 85/100/18778.3, PI 85/100/18778.3 (C's
operator-proxy and PI rows are identical because both independently saturate
the choke to 100% under the same noise seed, not a bug); D as above.

**results.json / generate_docs.py single-source-of-truth architecture.**
scenarios.py, baselines.py, verify_identification.py, and seed_sweep.py each
write their key numbers into outputs/results.json (via results_io.py's
update_results(section, data) merge-write helper) instead of numbers being
hand-typed into README.md / docs/report.md / docs/presentation.md separately
and drifting out of sync with each other (which is exactly what had happened
-- report.md and presentation.md were found retyped independently and had
gone stale in different places). generate_docs.py renders
`<!-- GENERATED:key --> ... <!-- END GENERATED -->` marker blocks in each doc
from that one JSON file, leaving hand-written analysis prose untouched.
Currently wired: identification_table, correction_table, safety_limits_table,
scenario_key_results_table, actuator_activity_table,
baseline_comparison_table_abc (A/B/C only), seed_sweep_table -- used in
README.md (1 block) and docs/report.md (6 blocks). Scenario D is NOT sourced
here: scenario_d.py doesn't write to results.json (a 200h run per approach is
a different cost profile than the other scripts; not yet worth the wiring),
so its numbers are hand-maintained wherever they appear (docs/report.md §3.6,
docs/presentation.md), re-verified against a fresh `python scenario_d.py` run
before being typed in each time, not carried over from memory.
docs/presentation.md has no markers at all (a slide deck's tables are
one-off per slide, not worth the wiring at this scope) -- its numbers are
likewise hand-maintained but checked against the same results.json/report.md
before publishing.

- docs/report.md, docs/presentation.md, and README.md are now fully resynced
  against one fresh pipeline run (identify -> verify_identification ->
  scenarios -> baselines -> seed_sweep -> scenario_d), including the Tier-0
  fix, the move-suppression fix and its actuator-activity metrics, Scenario D,
  the Fixed-operator-proxy baseline, and WHT/AP in every document.
  docs/presentation.md was rewritten end to end (14 -> 18 slides). Exported to
  docs/report.pdf, docs/report.pptx, docs/presentation.pdf,
  docs/presentation.pptx via export_report.py (pandoc -> self-contained HTML
  -> headless Chrome/Edge print, xelatex fallback) -- all four committed.
  Full pipeline re-run confirmed reproducible: byte-identical results.json
  except the newly-added seed_sweep section, all 41 tests pass.

TODO: investigate whether a larger noise_margin_sigma resolves Scenario C's
remaining chatter (move count barely improved, ~24% fallback rate); investigate
whether a scenario with real cross-channel coupling would let Scenario D
actually differentiate MPC from Fixed-optimal; wire Scenario D into
results.json if its cost profile ever stops being the blocker; final polish
pass.
