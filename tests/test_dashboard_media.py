"""Guard against displaying a rerender as a different recorded experiment."""

import json

import pytest

from dashboard.media import outcomes_match, render_track


def test_replay_requires_same_outcome_and_uncensored_distance():
    expected = {"stopped": True, "collision": False, "censored": False, "lane_departure": False,
                "stopping_distance": 62.4, "trial_duration": 7, "impact_speed": None,
                "final_front_x": 64.4, "final_speed": .02}
    assert outcomes_match(dict(expected), expected)
    assert not outcomes_match({**expected, "stopping_distance": 55.4}, expected)
    assert not outcomes_match({**expected, "collision": True}, expected)
    assert not outcomes_match({**expected, "stopping_distance": None}, expected)
    assert not outcomes_match({**expected, "impact_speed": 0}, expected)


def test_render_never_runs_an_unfinished_evaluation(tmp_path):
    (tmp_path / "metadata.json").write_text(json.dumps({"status": "evaluating"}))
    with pytest.raises(ValueError, match="completed"):
        render_track(tmp_path, "reserved_2", "reference")


def test_render_rejects_source_changed_after_freeze(tmp_path):
    (tmp_path / "metadata.json").write_text(json.dumps({"status": "completed"}))
    case = tmp_path / "evaluation/cases/reserved_1"
    case.mkdir(parents=True)
    (tmp_path / "evaluation/result.json").write_text(json.dumps({"source_sha256": "frozen-hash"}))
    (case / "candidate.json").write_text(json.dumps({"source_sha256": "frozen-hash"}))
    (tmp_path / "evaluation/frozen_candidate.py").write_text("raise RuntimeError('must never execute')\n")
    with pytest.raises(ValueError, match="does not match"):
        render_track(tmp_path, "reserved_1", "candidate")
