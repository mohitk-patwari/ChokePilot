"""
Single shared outputs/results.json that scenarios.py, baselines.py, and
verify_identification.py each write their own top-level section into -- the fix for
README.md/docs/report.md/docs/presentation.md's numbers disagreeing with each other
and with the shipped CSVs, since generate_docs.py renders all three from this one
file instead of anyone hand-typing a number into prose.

update_results() merges (not overwrites) so the three scripts can run in any order,
or just one at a time, without erasing what the others already wrote.
"""

import json
from pathlib import Path

RESULTS_PATH = Path(__file__).parent / "outputs" / "results.json"


def update_results(section, data):
    RESULTS_PATH.parent.mkdir(exist_ok=True)
    results = json.loads(RESULTS_PATH.read_text()) if RESULTS_PATH.exists() else {}
    results[section] = data
    RESULTS_PATH.write_text(json.dumps(results, indent=2, sort_keys=True))


def load_results():
    return json.loads(RESULTS_PATH.read_text())
