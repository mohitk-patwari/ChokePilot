"""
Regenerates the numeric tables in README.md, docs/report.md, and docs/presentation.md
from outputs/results.json -- the single source scenarios.py, baselines.py, and
verify_identification.py write into (see results_io.py). This is the fix for those
three documents' numbers disagreeing with each other and with the shipped CSVs: every
table below is rendered from the same JSON, not retyped by hand in three places.

Each doc keeps its hand-written analysis prose untouched. Only the content between
matched `<!-- GENERATED:key -->` / `<!-- END GENERATED -->` marker comments is
replaced, so this never touches narrative reasoning, only tables/inline numbers that
actually came from a pipeline run. Scenario D and the 30-seed sweep are NOT sourced
here -- only scenarios.py/baselines.py/verify_identification.py write to results.json
(scenario_d.py and seed_sweep.py don't), so those sections remain hand-maintained.
"""

import re
from pathlib import Path

from results_io import load_results

DOCS = [
    Path(__file__).parent / "README.md",
    Path(__file__).parent / "docs" / "report.md",
    Path(__file__).parent / "docs" / "presentation.md",
]

SCENARIO_ORDER = ["A", "B", "C"]
APPROACH_ORDER = ["MPC", "Fixed-optimal", "Fixed-operator-proxy", "PI"]

_LIMIT_BASIS = {
    "WHP": ('floor only (`hi = +inf`)',
            '*"If WHP becomes too low, the well may operate outside its recommended '
            'operating envelope."* High WHP just means the choke is closed back further '
            '— safe, not a hazard.'),
    "BHP": ('floor only (`hi = +inf`)',
            'Brief calls it *"one of the most important indicators of reservoir health '
            'and drawdown"* — low BHP means excessive drawdown (sand/formation-damage '
            'risk); high BHP means low drawdown, i.e. safely choked back.'),
    "FLP": ('ceiling only (`lo = -inf`)',
            'Brief: *"helps ensure stable transportation of produced fluids"* — the risk '
            'is backpressure/separator overpressure on the high side, not a low reading.'),
}


def fmt_limit(v):
    return f"{v:.0f}" if v is not None else "unbounded"


def identification_table(results):
    rows = ["| Channel | True τ (h) | Mean identified τ (h) | Mean error | Error range |",
            "|---|---|---|---|---|"]
    for ch, d in results["identification"]["channels"].items():
        rows.append(f"| {ch} | {d['true_tau']:.2f} | {d['ident_tau_mean']:.2f} | "
                    f"{d['tau_error_pct_mean']:.1f}% | "
                    f"{d['tau_error_pct_min']:.1f}–{d['tau_error_pct_max']:.1f}% |")
    return "\n".join(rows)


def correction_table(results):
    rows = ["| Channel | Physics-only RMSE | +Correction RMSE | Kept? |", "|---|---|---|---|"]
    for ch, d in results["correction"].items():
        mark = "✅ **used**" if d["used"] else "❌ skipped"
        rows.append(f"| {ch} | {d['physics_rmse']:.2f} | {d['corrected_rmse']:.2f} | {mark} |")
    return "\n".join(rows)


def safety_limits_table(results):
    rows = ["| Channel | Direction enforced | Limit | Brief basis |", "|---|---|---|---|"]
    for ch in ("WHP", "BHP", "FLP"):
        lo, hi = results["safety_limits"][ch]
        direction, basis = _LIMIT_BASIS[ch]
        limit = f"≥ {fmt_limit(lo)} psi" if hi is None else f"≤ {fmt_limit(hi)} psi"
        rows.append(f"| {ch} | {direction} | {limit} | {basis} |")
    return "\n".join(rows)


def scenario_key_results_table(results):
    """README-style: Scenario / Setup / Final / Violations."""
    rows = ["| Scenario | Setup | Final | Constraint violations |", "|---|---|---|---|"]
    for key in SCENARIO_ORDER:
        s = results["scenarios"][key]
        setup = s["label"].split(" - ", 1)[-1] if " - " in s["label"] else s["label"]
        rows.append(f"| {key} — {setup} | start {s['start_choke']:.1f}% choke, {s['hours']}h run "
                    f"| {s['final_q']:.1f} bbl/hr @ {s['final_choke']:.0f}% choke "
                    f"| {s['violations']}/{s['total_steps']} |")
    return "\n".join(rows)


def actuator_activity_table(results):
    """Move count and total valve travel per scenario -- the metric that catches
    chattering (many small moves) that violation counts and barrels alone don't."""
    rows = ["| Scenario | Moves | Total valve travel |", "|---|---|---|"]
    for key in SCENARIO_ORDER:
        s = results["scenarios"][key]
        rows.append(f"| {key} | {s['move_count']} / {s['total_steps']} "
                    f"| {s['total_travel_pct']:.1f} %-pts |")
    return "\n".join(rows)


def baseline_comparison_table_abc(results):
    """MPC/Fixed-optimal/Fixed-operator-proxy/PI x Scenarios A/B/C. Scenario D is not
    in results.json (scenario_d.py doesn't write to it) and is appended by hand where
    this table is used."""
    rows = ["| Scenario | Approach | Safety violations | Total barrels |", "|---|---|---|---|"]
    for key in SCENARIO_ORDER:
        for approach in APPROACH_ORDER:
            d = results["baselines"][key][approach]
            rows.append(f"| {key} | {approach} | {d['violations']}/{results['scenarios'][key]['total_steps']} "
                        f"| {d['barrels']:,.1f} |")
    return "\n".join(rows)


RENDERERS = {
    "identification_table": identification_table,
    "correction_table": correction_table,
    "safety_limits_table": safety_limits_table,
    "scenario_key_results_table": scenario_key_results_table,
    "actuator_activity_table": actuator_activity_table,
    "baseline_comparison_table_abc": baseline_comparison_table_abc,
}

_MARKER_RE = re.compile(r"<!-- GENERATED:(\w+) -->.*?<!-- END GENERATED -->", re.DOTALL)


def render_doc(path, results):
    text = path.read_text(encoding="utf-8")

    def replace(m):
        key = m.group(1)
        if key not in RENDERERS:
            raise KeyError(f"{path}: no renderer registered for GENERATED:{key}")
        return f"<!-- GENERATED:{key} -->\n{RENDERERS[key](results)}\n<!-- END GENERATED -->"

    new_text, n = _MARKER_RE.subn(replace, text)
    path.write_text(new_text, encoding="utf-8")
    print(f"{path}: regenerated {n} block(s)")


def main():
    results = load_results()
    for doc in DOCS:
        render_doc(doc, results)


if __name__ == "__main__":
    main()
