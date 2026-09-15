"""Receive-only adaptation of automated-lle's URArm (Xiaoman Guo).

Source: components/robot_move/ur5_rtde_gripper.py at 83a5169630793e45499de7b0043706b34abe2f90.
See docs/LLE_RTDE.md and docs/APACHE-2.0.md for provenance and license.
Modified: lazy receive-only connection, validated samples, no control/gripper
construction, no LLE positions, global component manager, or homing routines.
"""

from __future__ import annotations

import math
import threading
import time


def vector6(values):
    values = list(values)
    if len(values) != 6 or any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
        for v in values
    ):
        raise ValueError("Expected six finite numeric coordinates")
    return [float(v) for v in values]


def tcp_to_mm_deg(values):
    # Preserve URArm's convention: UR axis-angle radians -> Euler xyz degrees,
    # NOT six independently scaled values (Euler angles are not a rotvec).
    from scipy.spatial.transform import Rotation

    values = vector6(values)
    return [v * 1000 for v in values[:3]] + Rotation.from_rotvec(values[3:]).as_euler(
        "xyz", degrees=True
    ).tolist()


class URArm:
    """LLE-compatible joint/TCP reads, with no motion-capable connection.

    Constructing this object does not import an SDK or connect. connect() opens
    only RTDEReceiveInterface. There is deliberately no rtde_c or gripper.
    All operations are serialized, including shutdown and failure cleanup.
    """

    def __init__(self, robot_ip, *, frequency=50.0, timeout=2.0, receiver_factory=None):
        if not robot_ip or not math.isfinite(frequency) or not 1 <= frequency <= 50:
            raise ValueError("Explicit host and receive frequency 1..50 Hz required")
        if not math.isfinite(timeout) or not 0.1 <= timeout <= 5:
            raise ValueError("Sample timeout must be 0.1..5 seconds")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.timeout = timeout
        self._factory = receiver_factory
        self.rtde_r = None
        self._last_timestamp = None
        self._lock = threading.RLock()

    def connect(self):
        with self._lock:
            if self.rtde_r is not None:
                return
            factory = self._factory
            if factory is None:
                from rtde_receive import RTDEReceiveInterface

                factory = RTDEReceiveInterface
            self.rtde_r = factory(
                self.robot_ip,
                frequency=self.frequency,
                variables=["timestamp", "actual_q", "actual_TCP_pose"],
            )
            self._last_timestamp = None

    def _connected(self):
        if self.rtde_r is None or not self.rtde_r.isConnected():
            raise ConnectionError("RTDE receive stream is not connected")

    def get_joints(self):
        with self._lock:
            self._connected()
            return [math.degrees(j) for j in vector6(self.rtde_r.getActualQ())]

    def get_tcp_pose(self):
        with self._lock:
            self._connected()
            return tcp_to_mm_deg(self.rtde_r.getActualTCPPose())

    @property
    def joint_positions(self):
        return self.get_joints()

    def _timestamp(self):
        value = self.rtde_r.getTimestamp()
        if isinstance(value, bool) or not math.isfinite(value) or value < 0:
            raise ValueError("Invalid RTDE controller timestamp")
        return value

    def read(self):
        with self._lock:
            try:
                self.connect()
                self._connected()
                # Require a new controller packet, including after initial
                # connection. isConnected() alone cannot establish freshness.
                baseline = self._timestamp()
                if self._last_timestamp is not None and baseline < self._last_timestamp:
                    raise ConnectionError("RTDE controller clock restarted")
                deadline = time.monotonic() + self.timeout
                while True:
                    self._connected()
                    timestamp = self._timestamp()
                    if timestamp < baseline:
                        raise ConnectionError("RTDE controller clock restarted")
                    if timestamp > baseline:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("RTDE receive stream stopped advancing")
                    time.sleep(min(1 / self.frequency, 0.02))
                joints = self.get_joints()
                raw_pose = vector6(self.rtde_r.getActualTCPPose())
                pose = tcp_to_mm_deg(raw_pose)
                self._connected()
                end_timestamp = self._timestamp()
                if (
                    end_timestamp < timestamp
                    or end_timestamp - timestamp > self.timeout
                ):
                    raise ConnectionError(
                        "RTDE sample changed clock or exceeded read window"
                    )
                self._last_timestamp = end_timestamp
                return {
                    "valid": True,
                    "source": "rtde_receive",
                    "controller_timestamp_s": timestamp,
                    "joints_deg": vector6(joints),
                    "tcp_mm_rpy_deg": vector6(pose),
                    "tcp_m_rotvec_rad": raw_pose,
                }
            except Exception:
                self.disconnect()
                raise

    def disconnect(self):
        with self._lock:
            receiver, self.rtde_r = self.rtde_r, None
            self._last_timestamp = None
            if receiver is not None:
                receiver.disconnect()
