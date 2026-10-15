"""Status presentation for safeguard recovery and the CB3 control script.

Dashboard programState is a fixture input. Nothing here opens a socket.
"""

import time

from fastapi.testclient import TestClient

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.ur import interpret, present_status, work_in_flight


def _view(**flags):
    base = dict(
        control_enabled=True,
        session_open=False,
        watchdog_ok=False,
        watchdog_error=None,
    )
    base.update(flags)
    return base


def test_present_status_does_not_mutate_the_cached_observation():
    observed = interpret("RUNNING", "NORMAL", "STOPPED")
    observed["details"]["telemetry"] = {"valid": True, "joints_deg": [1, 0, 0, 0, 0, 0]}
    view = present_status(observed, **_view(session_open=True, watchdog_ok=True))
    assert observed["details"]["program_state"] == "STOPPED"
    assert observed["equipment_status"] == "ready"
    assert "program_state_source" not in observed["details"]
    assert view["details"]["program_state"] == "PLAYING"
    assert view["details"]["program_state_source"] == "control_session"
    assert view["details"]["telemetry"]["joints_deg"][0] == 1
    # The script is up; the arm is not moving. ready ⇒ idle.
    assert view["equipment_status"] == "ready"
    assert view["activity"] == "idle"
    assert view["message"] == "Read-only observation: program playing"
    assert view["required_actions"] == []
    assert "external script" in view["components"]["program"].message


def test_an_already_playing_dashboard_program_is_left_alone():
    """A loaded program Dashboard already calls PLAYING is still that program."""
    observed = interpret("RUNNING", "NORMAL", "PLAYING")
    view = present_status(observed, **_view(session_open=True, watchdog_ok=True))
    assert view["details"]["program_state"] == "PLAYING"
    assert "program_state_source" not in view["details"]
    assert view["components"]["program"].message is None
    assert view["equipment_status"] == "busy"
    assert view["activity"] == "running"


def test_reduced_mode_still_reports_the_control_script():
    observed = interpret("RUNNING", "REDUCED", "STOPPED")
    view = present_status(observed, **_view(session_open=True, watchdog_ok=True))
    assert view["equipment_status"] == "degraded"
    assert view["activity"] == "idle"
    assert view["details"]["program_state"] == "PLAYING"
    assert "program PLAYING" in view["message"]
    moving = present_status(
        observed, **_view(session_open=True, watchdog_ok=True, motion_active=True)
    )
    assert moving["equipment_status"] == "degraded"
    assert moving["activity"] == "running"
    assert moving["details"]["program_state"] == "PLAYING"


def test_a_safety_fault_is_not_rewritten_as_a_playing_script_or_a_reconnect():
    for safety, state in (
        ("PROTECTIVE_STOP", "error"),
        ("ROBOT_EMERGENCY_STOP", "e_stop"),
    ):
        observed = interpret("RUNNING", safety, "STOPPED")
        for watchdog_ok in (True, False):
            view = present_status(
                observed,
                **_view(
                    session_open=True,
                    watchdog_ok=watchdog_ok,
                    watchdog_error=(
                        None
                        if watchdog_ok
                        else "Controller did not acknowledge the watchdog kick"
                    ),
                ),
            )
            assert view["equipment_status"] == state
            assert view["details"]["program_state"] == "STOPPED"
            assert view["required_actions"] == []
            assert view["last_error"] is not None


def test_safeguard_with_an_open_session_asks_for_disconnect_then_connect():
    observed = interpret("RUNNING", "SAFEGUARD_STOP", "PLAYING")
    for watchdog_ok in (True, False):
        view = present_status(
            observed,
            **_view(
                session_open=True,
                watchdog_ok=watchdog_ok,
                watchdog_error=(
                    None
                    if watchdog_ok
                    else "Controller did not acknowledge the watchdog kick"
                ),
            ),
        )
        assert view["equipment_status"] == "requires_init"
        assert view["activity"] == "idle"
        assert view["last_error"] is None
        assert view["required_actions"] == ["disconnect", "connect"]
        assert view["message"].startswith("Safeguard stop (SAFEGUARD_STOP)")
        assert (
            "Disconnect and connect once the safety stop is released."
            in view["message"]
        )
        # The external-script correction must not hide the stop.
        assert view["details"]["program_state"] == "PLAYING"
        assert "program_state_source" not in view["details"]
        assert view["control_message"] == view["message"]


def test_safeguard_without_a_session_names_connect_or_the_release():
    observed = interpret("RUNNING", "AUTOMATIC_MODE_SAFEGUARD_STOP", "STOPPED")
    armed = present_status(observed, **_view())
    assert armed["required_actions"] == ["connect"]
    assert "Disconnect" not in armed["message"]
    readonly = present_status(observed, **_view(control_enabled=False))
    assert readonly["required_actions"] == ["release_safety_stop"]
    assert readonly["equipment_status"] == "requires_init"
    assert readonly["activity"] == "idle"


def test_a_dead_session_on_a_normal_robot_is_requires_init_not_ready():
    observed = interpret("RUNNING", "NORMAL", "STOPPED")
    view = present_status(
        observed,
        **_view(
            session_open=True,
            watchdog_ok=False,
            watchdog_error="Controller did not acknowledge the watchdog kick",
        ),
    )
    assert view["equipment_status"] == "requires_init"
    assert view["activity"] == "idle"
    assert view["required_actions"] == ["disconnect", "connect"]
    assert "Controller did not acknowledge the watchdog kick" in view["message"]
    assert "Disconnect and connect to resume." in view["message"]
    assert view["details"]["program_state"] == "STOPPED"
    assert "program_state_source" not in view["details"]
    # The Dashboard reading underneath is still ready; only the view changes.
    assert observed["equipment_status"] == "ready"


def test_controller_power_off_names_connect_only_when_control_exists():
    observed = interpret("POWER_OFF", "NORMAL", "STOPPED")
    assert present_status(observed, **_view())["required_actions"] == ["connect"]
    assert (
        present_status(observed, **_view(control_enabled=False))["required_actions"]
        == []
    )
    held = present_status(
        observed, **_view(session_open=True, watchdog_ok=False, watchdog_error="gone")
    )
    assert held["equipment_status"] == "requires_init"
    assert held["required_actions"] == ["disconnect", "connect"]
    assert held["message"].startswith("Controller mode: POWER_OFF")
    assert "Disconnect and connect to resume." in held["message"]


def test_in_flight_work_is_running_and_a_connected_arm_stays_idle():
    observed = interpret("RUNNING", "NORMAL", "STOPPED")
    idle = present_status(observed, **_view(session_open=True, watchdog_ok=True))
    assert idle["details"]["program_state"] == "PLAYING"
    assert idle["activity"] == "idle"
    assert idle["equipment_status"] == "ready"
    for flags in (
        dict(arm_busy=True),
        dict(active_move={"kind": "joint"}),
        dict(trajectory_state="running"),
        dict(trajectory_state="starting"),
        dict(trajectory_state="stopping"),
        dict(gripper_busy=True),
    ):
        assert work_in_flight(**flags) is True
        view = present_status(
            observed, **_view(session_open=True, watchdog_ok=True, motion_active=True)
        )
        assert view["activity"] == "running"
        assert view["equipment_status"] == "busy"
        assert view["details"]["program_state"] == "PLAYING"
    for quiet in (None, "created", "completed", "stopped"):
        assert work_in_flight(trajectory_state=quiet) is False


def test_readonly_service_reports_the_safeguard_release():
    class Observer:
        def read(self):
            return interpret("RUNNING", "SAFEGUARD_STOP", "STOPPED")

    settings = Settings(
        driver="ur", model="ur5_cb3", observe=True, robot_host="robot.invalid"
    )
    with TestClient(create_app(settings, observer=Observer())) as client:
        deadline_status = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            deadline_status = client.get("/status").json()
            if deadline_status["details"].get("safetystatus") == "SAFEGUARD_STOP":
                break
            time.sleep(0.02)
        assert deadline_status["equipment_status"] == "requires_init"
        assert deadline_status["activity"] == "idle"
        assert deadline_status["required_actions"] == ["release_safety_stop"]
        assert deadline_status["allowed_actions"] == []
        assert "control" not in deadline_status["components"]
        assert client.post("/connect", json={}).status_code == 404
