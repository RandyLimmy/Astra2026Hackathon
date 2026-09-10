from investigation.platform_story import Story, criteria


def test_diagnosis_and_failed_attempt_are_not_applied_fixes(tmp_path):
    story = Story(tmp_path, "drone", criteria("drone", ["hover", "maneuver"], 10))
    story.initial_mismatch = True
    story.explanation("The front rotor needs repair.")
    assert story.action_summary()["status"] == "diagnosed_but_no_fix_attempted"
    event = story.begin("apply_repair", {"action": "replace_rotor", "target": "bad_target"})
    story.finish(event, {"ok": False, "error": "Choose a declared target."})
    assert story.action_summary()["fix_attempted"] is True
    assert story.action_summary()["change_applied"] is False
    assert story.action_summary()["failed_attempts"] == 1


def test_source_edits_and_measured_repairs_remain_distinct(tmp_path):
    story = Story(tmp_path, "drone", criteria("drone", ["hover"], 10))
    edit = story.begin("replace_model_source", {"rationale": "Model the lost thrust."})
    story.finish(edit, {"ok": True, "accepted": True}, source_diff="-gain = 1\n+gain = .5\n")
    assert story.action_summary()["repairs_applied"] == 0
    assert story.action_summary()["model_edits_applied"] == 1
    repair = story.begin("apply_repair", {"action": "replace_rotor", "target": "rotor_FL"})
    story.finish(repair, {"ok": True}, before={"rotor_FL_gain": .12}, after={"rotor_FL_gain": 1.})
    assert repair["changes"] == [{"parameter": "rotor_FL_gain", "before": .12, "after": 1.}]
    assert story.action_summary()["status"] == "applied_unverified"
    check = story.begin("check_repair", {"probe": "hover"})
    story.finish(check, {"ok": True, "difference": {"observed_within_envelope": True}})
    assert check["stage"] == "verified"
    assert story.verification["goal_achieved"] is None  # One development check is not the final goal.


def test_replay_index_cannot_point_outside_its_run(tmp_path):
    from types import SimpleNamespace
    story = Story(tmp_path, "car", criteria("car", ["steering"], 10))
    story.replay(SimpleNamespace(workdir=tmp_path / "physics"), {"id": "../../other"}, "before", "Original")
    assert not story.replays
