"""Owned RTDE control session and fresh feedback for the joint-step routes.

Nothing here runs at import or at service start. ControlSession.open() is the
one place that constructs RTDEControlInterface, which uploads ur_rtde's control
script to the controller and takes over its program slot. The UI's 15 s poll is
far too old for control, so the session also owns a dedicated receive stream
that includes joint velocities and the controller/safety modes. Tests inject
fakes; the vendor SDK is imported only inside the default factories.
"""

from __future__ import annotations

import math
import threading
import time

from .lle_rtde import vector6

# UR controller enumerations as delivered by the RTDE robot_mode/safety_mode
# outputs. Unknown values are reported, never mapped onto a permissive name.
ROBOT_MODES = {
    -1: "NO_CONTROLLER",
    0: "DISCONNECTED",
    1: "CONFIRM_SAFETY",
    2: "BOOTING",
    3: "POWER_OFF",
    4: "POWER_ON",
    5: "IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}
SAFETY_MODES = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY",
    5: "SAFEGUARD_STOP",
    6: "SYSTEM_EMERGENCY_STOP",
    7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION",
    9: "FAULT",
    10: "VALIDATE_JOINT_ID",
    11: "UNDEFINED_SAFETY_MODE",
    12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
    13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
}
FEEDBACK_VARIABLES = ["timestamp", "actual_q", "actual_qd", "robot_mode", "safety_mode"]


def mode_name(table, value):
    try:
        number = int(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    if isinstance(value, bool):
        return "UNKNOWN"
    return table.get(number, f"UNKNOWN_{number}")


class ControlFeedback:
    """Latest controller packet, stamped with host time when it was first seen.

    ur_rtde does not expose packet receipt times, so the poll thread runs at
    twice the stream frequency and stamps a sample only when the controller
    timestamp advances. That bounds the stamp error to one poll interval.
    """

    def __init__(self, receiver, *, frequency_hz, clock=time.monotonic, sleep=time.sleep):
        self.receiver = receiver
        self.frequency_hz = frequency_hz
        self.clock = clock
        self.sleep = sleep
        self._lock = threading.Lock()
        self._fresh = threading.Condition(self._lock)
        self._latest = None
        self._error = None
        self._served = None
        self._stop = threading.Event()
        self._thread = None

    def poll_once(self):
        receiver = self.receiver
        connected = bool(receiver.isConnected())
        timestamp = float(receiver.getTimestamp())
        with self._lock:
            previous = self._latest
        if previous is not None and timestamp <= previous["controller_timestamp_s"] and connected:
            return False
        sample = {
            "joints_deg": tuple(math.degrees(q) for q in vector6(receiver.getActualQ())),
            "velocities_deg_s": tuple(math.degrees(v) for v in vector6(receiver.getActualQd())),
            "controller_timestamp_s": timestamp,
            "received_monotonic_s": self.clock(),
            "controller_connected": connected,
            "robot_mode": mode_name(ROBOT_MODES, receiver.getRobotMode()),
            "safety_mode": mode_name(SAFETY_MODES, receiver.getSafetyMode()),
        }
        with self._lock:
            self._latest = sample
            self._error = None
            self._fresh.notify_all()
        return True

    def _run(self):
        interval = 1 / (2 * self.frequency_hz)
        while not self._stop.is_set():
            try:
                self.poll_once()
            except Exception as exc:  # a broken stream must read as absent, not old
                with self._lock:
                    self._latest = None
                    self._error = str(exc)
                    self._fresh.notify_all()
            self.sleep(interval)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="ur-control-feedback", daemon=True)
            self._thread.start()

    def read(self, wait_s=0.2):
        """Return a packet newer than the last one served, waiting at most wait_s.

        The executor treats every read as a distinct controller packet, so a
        repeat of the previous packet must not be handed back as if it were
        new. When nothing newer arrives in time the latest packet is returned
        and the executor's own advancing/stale checks refuse the step.
        """
        deadline = time.monotonic() + wait_s
        with self._fresh:
            while True:
                latest, error = self._latest, self._error
                if latest is not None and (
                    self._served is None or latest["controller_timestamp_s"] > self._served
                ):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._fresh.wait(remaining)
            if latest is None:
                raise ConnectionError(error or "No control feedback sample yet")
            self._served = latest["controller_timestamp_s"]
            return dict(latest)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self.receiver.disconnect()


class _SerializedControl:
    """RTDEControlInterface is not thread-safe: one caller inside the SDK at a time.

    fault is set by the watchdog kicker when the controller stops acknowledging
    kicks; from then on the interface reports disconnected so the executor
    refuses rather than trusting a link the controller may have dropped.
    """

    def __init__(self, control, lock):
        self._control = control
        self._lock = lock
        self.fault = None

    def isConnected(self):
        if self.fault is not None:
            return False
        with self._lock:
            return bool(self._control.isConnected())

    def setWatchdog(self, min_frequency):
        with self._lock:
            return self._control.setWatchdog(min_frequency)

    def kickWatchdog(self):
        with self._lock:
            return self._control.kickWatchdog()

    def moveJ(self, target, speed, acceleration, asynchronous):
        with self._lock:
            return self._control.moveJ(target, speed, acceleration, asynchronous)

    def stopJ(self, deceleration):
        with self._lock:
            return self._control.stopJ(deceleration)

    def disconnect(self):
        with self._lock:
            return self._control.disconnect()


def _default_receiver(host, **kwargs):
    from rtde_receive import RTDEReceiveInterface

    return RTDEReceiveInterface(host, **kwargs)


def _default_control(host):
    # Default flags upload the ur_rtde control script: this takes the
    # controller's program slot. Never construct it passively.
    from rtde_control import RTDEControlInterface

    return RTDEControlInterface(host)


class ControlSession:
    def __init__(self, settings, *, control_factory=None, receiver_factory=None, clock=time.monotonic):
        self.settings = settings
        self._control_factory = control_factory or _default_control
        self._receiver_factory = receiver_factory or _default_receiver
        self.clock = clock
        self._lock = threading.RLock()
        self._sdk_lock = threading.RLock()
        self.control = None
        self.feedback = None
        self._kicker = None
        self._kick_stop = threading.Event()
        self.watchdog_error = None

    @property
    def is_open(self):
        return self.control is not None

    @property
    def watchdog_ok(self):
        return self.control is not None and self.watchdog_error is None

    def _kick(self, proxy, interval):
        # The controller-side watchdog is the fail-safe for a frozen or dead
        # service: once kicks stop, the controller halts the control script.
        # A refused kick is treated the same way here: stop kicking, mark the
        # interface faulted, and let the controller do its part.
        while not self._kick_stop.wait(interval):
            try:
                if proxy.kickWatchdog() is not True:
                    raise RuntimeError("Controller did not acknowledge the watchdog kick")
            except Exception as exc:
                self.watchdog_error = str(exc)
                proxy.fault = f"watchdog: {exc}"
                return

    def open(self):
        with self._lock:
            if self.control is not None:
                raise RuntimeError("Control session already open")
            frequency = self.settings.control.feedback_frequency_hz
            watchdog_hz = self.settings.control.watchdog_hz
            receiver = self._receiver_factory(
                self.settings.robot_host, frequency=frequency, variables=list(FEEDBACK_VARIABLES)
            )
            feedback = ControlFeedback(receiver, frequency_hz=frequency, clock=self.clock)
            feedback.start()
            try:
                control = self._control_factory(self.settings.robot_host)
            except Exception:
                feedback.close()
                raise
            proxy = _SerializedControl(control, self._sdk_lock)
            try:
                if proxy.setWatchdog(watchdog_hz) is not True:
                    raise RuntimeError("Controller refused the communication watchdog")
            except Exception:
                proxy.disconnect()
                feedback.close()
                raise
            self.watchdog_error = None
            self._kick_stop.clear()
            self._kicker = threading.Thread(
                target=self._kick, args=(proxy, 1 / (2 * watchdog_hz)),
                name="ur-control-watchdog", daemon=True,
            )
            self._kicker.start()
            self.feedback = feedback
            self.control = proxy

    def stop(self, deceleration_deg_s2):
        with self._lock:
            if self.control is None:
                return False
            self.control.stopJ(math.radians(deceleration_deg_s2))
            return True

    def close(self):
        with self._lock:
            control, feedback, kicker = self.control, self.feedback, self._kicker
            self.control = self.feedback = self._kicker = None
        self._kick_stop.set()
        if kicker is not None:
            kicker.join(timeout=2)
        errors = []
        for closer in (
            getattr(control, "disconnect", None),
            getattr(feedback, "close", None),
        ):
            if closer is None:
                continue
            try:
                closer()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))
