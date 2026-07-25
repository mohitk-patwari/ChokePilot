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
  violation (30/30), mean 2.67/80 steps, max 4/80. Unaffected by the
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
  extrapolation; B/C start fully inside the calibrated band).
- Hybrid correction layer validated per-channel on held-out data via
  select_beneficial_corrections(): currently only BHP's correction generalizes
  and is used; Q/WHP/FLP run physics-only.
- DWELL_HOURS 24->40 fix, confirmed (not just hypothesized) as the cause of
  most tau identification bias: WHP/FLP/BHP mean tau error dropped
  16%/27%/31% -> 4%/14%/8%. Q's ~19-20% bias is unaffected across every dwell
  tested (24-80h) and remains open (see TODO).
- WHT (wellhead temperature) and AP (annulus pressure) added to
  data/simulator.py as monitored-but-not-constrained channels -- hand-set
  placeholder params (no CSV column exists for either), never feed the
  controller's safety check or MPC objective, plotted greyed-out in
  scenarios.py, read via Simulator.read_monitored().
- seed_sweep.py: 30-seed distribution sweep per scenario (violation counts,
  safety-fallback frequency) -- the source of the honest numbers now used
  throughout docs/report.md and this file, in place of single-run numbers.
- baselines.py: fixed-choke and IMC-tuned PI baselines vs. the MPC, same
  scenarios/model/limits/seeds. Headline result: the safety-blind PI baseline
  racks up 169 constraint-violation samples chasing Scenario C's infeasible
  target; the MPC and fixed baselines both stay safe.
- requirements.txt (pinned numpy/pandas/matplotlib/pytest) and tests/ (41
  pytest tests: ramp-rate/choke-bound actuator constraints, identified-tau
  thresholds across 5 seeds, infeasible-target safety across the 30-seed
  sweep) -- run as the safety gate before every push.
- docs/report.md rewritten from scratch against a fresh DWELL_HOURS=40
  pipeline run (not carried over from any earlier draft): fixes the "treats
  the simulator as a black box" framing (identify.py imports simulator.py's
  private fitting functions directly, so model structure is shared by
  construction), renames "brute-force MPC" to "one-step receding-horizon
  search with hold-constant prediction" and states plainly that safety is
  best-effort with no recursive feasibility check, retracts the false "15% is
  inside supported territory" claim, reconciles the pain-point count to five
  throughout, replaces single-sample endpoint claims with trailing-10h means,
  and adds the seed-sweep and baseline-comparison tables. Pushed.
- verify_identification.py fixed (KeyError from WHT/AP being included in
  simulator.PARAMS but never identified by identify.py -- restricted to
  identify.CHANNELS).

One-sample lag fix: _simulate_fopdt drove sim[k+1] from y_ss[k] (i.e. u[k-theta]);
Simulator.step() drives the arriving state from u[k+1-theta] (same time index as
the arrival sample). Fixed to y_ss[k+1] (also fixed identically in identify.py's
_simulate_with_correction, which had the same bug plus a missed Euler-vs-ZOH
inconsistency). This WAS Q's "unexplained ~19% tau bias" -- theta now matches
true theta exactly in 20/20 seed x channel combinations, Q's tau error dropped
to ~3%. Recalibrating simulator.py's own PARAMS with the fix also revealed the
"true" theta values were themselves off by 1 this whole time (now 5.00/1,
7.50/2, 7.00/1, 9.00/3 -- tau unchanged, theta all +1).

Move-suppression fix: MPCController picked candidates by q_err alone (tie-break
on move size only), so once near target, candidates within noise of each other
on q_err got picked by noise -- chattered the valve (Scenario C: 52 moves / 154
%-points travel over 100h). Added cost = q_err + LAMBDA_MOVE*abs(delta) (also
applied to the fallback branch's violation-based cost, same reasoning, since
Scenario C spends 16-23% of steps there). LAMBDA_MOVE=1.0 (bbl/hr required per
%-point moved). Result: Scenario A/B chattering resolved (5/80, 7/140 moves).
Scenario C only partially improved (travel 154->129, but move count 52->53
essentially unchanged) -- root cause is different: C rides the tightened WHP
floor (208.6 psi) with WHP noise (sigma~1.2 psi, observed swinging 207-213)
large enough to flip which candidates are feasible step to step, which a
cost-ranking fix within whichever set is feasible can't solve. A larger
noise_margin_sigma would likely help; not tried.

STALE, not yet refreshed against recent fixes: docs/presentation.md (still
references the old 24h dwell and 34.4% start) and README.md (still says
"treats it as a black box" and quotes pre-DWELL_HOURS-fix numbers) -- neither
was part of any pass so far. docs/report.md Sections 1.2/3.2/3.3/3.6 are also
stale (predate the one-sample-lag and/or move-suppression fixes); flagged
inline there. Sections 2.3/3.1/3.4/3.7 are current (2.3/3.1 rewritten from
scratch after finding an actual error, not just a numbers refresh -- see
Known Limitation above).

TODO: refresh docs/presentation.md and README.md against the current state;
resync docs/report.md's remaining stale sections (1.2/3.2/3.3/3.6); investigate
whether a larger noise_margin_sigma resolves Scenario C's remaining chatter;
final polish pass.
