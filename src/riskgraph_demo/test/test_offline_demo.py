"""End-to-end offline demo regression: the bundled glossy_hallway scenario must
choose the LONG route and produce an explanation that mentions slip."""
import json
from pathlib import Path

from riskgraph_demo.scenario import load_scenario
from riskgraph_demo.offline_demo import run_demo


_FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "scenario_glossy_hallway.json"


def test_glossy_hallway_scenario_chooses_long(tmp_path):
    out = tmp_path / "results.json"
    rc = run_demo(str(_FIXTURE), str(out))
    assert rc == 0, "demo regression failed: chosen route or explanation did not match expectations"
    data = json.loads(out.read_text())
    assert data["chosen_route_id"] == "LONG"
    assert data["expected_choice"] == "LONG"
    assert data["verdict"]["overall"] is True
    assert "slip" in data["explanation"]["text"].lower()
    # The explanation must cite at least one event that was actually recorded.
    assert len(data["explanation"]["evidence_event_ids"]) >= 1


def test_scenario_loader_round_trips_fixture():
    sc = load_scenario(str(_FIXTURE))
    assert sc.name == "glossy_hallway"
    assert {r.route_id for r in sc.routes} == {"SHORT", "LONG"}
    assert len(sc.events) == 4
    assert sc.weights.risk == 4.0
