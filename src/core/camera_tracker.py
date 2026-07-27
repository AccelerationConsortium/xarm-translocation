"""Pan the lab camera to follow the arm on the motion graph.

When the xArm arrives at a motion-graph node, its station tag (``uplc``,
``opentrons``, ``deck``, ``shaker``, ``filter`` — see
``src/settings/motion_graph.yaml``) is resolved to a named camera *view*,
and that view is commanded on the lab camera through the ac-organic-lab
dashboard control passthrough (the same surface the dashboard's web
``CameraTile`` uses):

    POST {dashboard_base_url}/api/equipment/{camera_id}/control/preset/goto
    POST {dashboard_base_url}/api/equipment/{camera_id}/control/ptz

Two independent switches gate the behaviour:

* **configured** — the feature is set up: ``enabled: true`` in
  ``src/settings/camera_tracking.yaml`` *and* a ``dashboard_base_url`` +
  ``camera_id`` are present. This decides whether the arm panel offers a
  camera card + "Follow arm" toggle at all.
* **following** — the runtime toggle behind that card. Only when *both*
  ``configured`` and ``following`` are true does an arrival actually pan
  the camera. ``following`` starts from ``follow_by_default`` and is
  flipped at runtime via :meth:`set_following` (wired to
  ``POST /camera/follow`` on the API server).

Separately, :meth:`availability` reports whether the camera is reachable
*right now* (reads the camera's live ``/status`` through the dashboard),
so the panel can hide/disable the toggle when the camera is off (privacy
mode, streaming disabled, unplugged) even while the feature is configured.

Design constraints, mirroring ``core/events_exporter.py``:

1. **Never block or break the control path.** ``notify_node`` resolves the
   view synchronously (cheap) and hands the HTTP POST to a daemon thread by
   default. Any failure — unreachable dashboard, non-2xx, bad config — is
   logged and swallowed; the arm keeps moving.
2. **Stdlib only.** ``urllib`` instead of a new HTTP dependency.
3. **Disabled unless configured.** ``enabled: false`` (or a missing config
   file, or a missing ``dashboard_base_url`` / ``camera_id``) makes the
   tracker an inert no-op, so dev machines, unit tests, and sims do nothing.

Configuration lives in ``src/settings/camera_tracking.yaml``; see that file
for the field documentation and the placeholders to fill in at the lab.
"""

from __future__ import annotations

import json
import threading
import time
import os
import urllib.error
import urllib.request
from typing import Any, Callable, Dict, Optional

# A sender takes (url, payload, headers, timeout_s) and raises on failure.
Sender = Callable[[str, Dict[str, Any], Dict[str, str], float], None]
# A fetcher takes (url, timeout_s) and returns parsed JSON, raising on failure.
Fetcher = Callable[[str, float], Any]

_DEFAULT_TIMEOUT_S = 5.0
# How long an availability probe result is reused before we re-hit the
# dashboard. The arm panel polls /camera/config every ~10-15s; this keeps
# that from becoming a per-poll round-trip to the dashboard.
_AVAILABILITY_TTL_S = 8.0


class CameraTracker:
    """Resolve motion-graph nodes to camera views and command them.

    ``sender`` is injectable for tests: a callable taking
    ``(url, payload, headers, timeout_s)`` and raising on delivery failure.
    When ``sender`` is left as the default, the POST is dispatched on a
    short-lived daemon thread (fire-and-forget); an injected sender is
    invoked synchronously so tests stay deterministic. ``fetcher`` is the
    read-side equivalent, used by :meth:`availability`.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]],
        *,
        sender: Optional[Sender] = None,
        fetcher: Optional[Fetcher] = None,
        environ: Optional[Dict[str, str]] = None,
    ):
        config = config or {}
        self._environ = environ if environ is not None else os.environ

        self.dashboard_base_url = str(config.get("dashboard_base_url", "") or "").strip().rstrip("/")
        self.camera_id = str(config.get("camera_id", "") or "").strip()
        # Which lens to stream/point (falls back to the first lens the
        # camera reports). PTZ is per-camera, so this only selects the video.
        self.lens = str(config.get("lens", "") or "").strip() or None
        # Optional distinct origin for the live video (go2rtc/Caddy). The
        # dashboard *API* (dashboard_base_url) and the *stream* origin can
        # differ; when unset we derive the stream origin from the API URL.
        self.stream_base_url = str(config.get("stream_base_url", "") or "").strip().rstrip("/")
        self._api_key_env = str(config.get("api_key_env", "") or "").strip()
        try:
            self._timeout_s = float(config.get("request_timeout_seconds", _DEFAULT_TIMEOUT_S))
        except (TypeError, ValueError):
            self._timeout_s = _DEFAULT_TIMEOUT_S

        views = config.get("views") or {}
        self._views: Dict[str, Dict[str, Any]] = {
            str(name): (body or {}) for name, body in views.items() if isinstance(body, dict)
        }
        stations = config.get("stations") or {}
        # Preserve YAML order: first matching tag wins for multi-tag nodes.
        self._stations: Dict[str, str] = {
            str(tag): str(view) for tag, view in stations.items()
        }

        # A test-injected sender runs synchronously; the default threads.
        self._sync_sender = sender
        self._sender: Sender = sender or self._threaded_send
        self._fetcher: Fetcher = fetcher or self._http_get_json

        # Dedupe: the last view we commanded, so hops within one station
        # (and re-planned travel through it) don't re-send.
        self._last_view: Optional[str] = None
        self._lock = threading.Lock()

        # Short-lived cache for the availability probe (see _AVAILABILITY_TTL_S).
        self._avail_cache: Optional[Dict[str, Any]] = None
        self._avail_cache_at: float = 0.0

        requested = bool(config.get("enabled", False))
        # Only truly *configured* when we also have somewhere to send.
        self.configured = requested and bool(self.dashboard_base_url) and bool(self.camera_id)
        if requested and not self.configured:
            print(
                "[camera] tracking requested but disabled: "
                "dashboard_base_url and camera_id are required"
            )

        # Runtime toggle. Defaults from follow_by_default (default True: if
        # you configured the camera you almost always want it to follow;
        # the panel toggle is there to pause it). Never on when unconfigured.
        follow_default = config.get("follow_by_default", True)
        self.following = bool(follow_default) and self.configured

    @classmethod
    def from_config_file(
        cls,
        path: str,
        *,
        sender: Optional[Sender] = None,
        fetcher: Optional[Fetcher] = None,
        environ: Optional[Dict[str, str]] = None,
    ) -> "CameraTracker":
        """Build from a YAML file. Missing/invalid file -> disabled no-op."""
        config: Dict[str, Any] = {}
        try:
            import yaml  # local import: keeps the module importable without PyYAML

            with open(path, "r") as handle:
                loaded = yaml.safe_load(handle)
            if isinstance(loaded, dict):
                config = loaded
        except FileNotFoundError:
            print(f"[camera] no config at {path}; camera tracking disabled")
        except Exception as exc:  # noqa: BLE001 - never break controller boot
            print(f"[camera] failed to load {path}: {exc}; camera tracking disabled")
        return cls(config, sender=sender, fetcher=fetcher, environ=environ)

    # ------------------------------------------------------------------
    # Runtime toggle
    # ------------------------------------------------------------------

    def set_following(self, value: bool) -> bool:
        """Turn "follow the arm" on/off at runtime. Returns the new state.

        Never enables when the feature isn't configured. Turning follow
        *on* resets the dedupe so the next arrival re-aims the camera even
        if it targets the same station the tracker last commanded.
        """
        new_value = bool(value) and self.configured
        with self._lock:
            if new_value and not self.following:
                self._last_view = None  # re-aim on the next arrival
            self.following = new_value
        return self.following

    # ------------------------------------------------------------------
    # Availability (read the camera's live state through the dashboard)
    # ------------------------------------------------------------------

    def availability(self, *, force: bool = False) -> Dict[str, Any]:
        """Report whether the camera is reachable/streamable right now.

        Returns a dict the API server hands to the panel::

            {configured, available, following, reason, stream_url, camera_id}

        Never raises: a probe failure yields ``available: False`` with a
        human ``reason``. Cached for ``_AVAILABILITY_TTL_S`` so the panel's
        poll doesn't become a per-request round-trip to the dashboard.
        """
        base = {
            "configured": self.configured,
            "following": self.following,
            "camera_id": self.camera_id or None,
        }
        if not self.configured:
            return {**base, "available": False, "reason": "camera tracking not configured", "stream_url": None}

        now = time.monotonic()
        if not force and self._avail_cache is not None and (now - self._avail_cache_at) < _AVAILABILITY_TTL_S:
            # Re-stamp the volatile fields; cache only the probe result.
            return {**self._avail_cache, **base}

        available, reason, stream_url = self._probe()
        self._avail_cache = {"available": available, "reason": reason, "stream_url": stream_url}
        self._avail_cache_at = now
        return {**base, **self._avail_cache}

    def _probe(self):
        """Return (available, reason, stream_url) from the dashboard snapshot."""
        url = f"{self.dashboard_base_url}/api/equipment"
        try:
            data = self._fetcher(url, self._timeout_s)
        except Exception as exc:  # noqa: BLE001 - best-effort read
            return False, f"dashboard unreachable: {exc}", None

        snap = self._find_camera_snapshot(data)
        if snap is None:
            return False, f"camera {self.camera_id!r} not found on dashboard", None
        if snap.get("fetch_error"):
            return False, "camera unreachable (dashboard fetch_error)", None

        # The dashboard's /api/equipment decorates each device as an
        # EquipmentSnapshot whose raw STATUS_SPEC envelope (equipment_status,
        # details.lenses, privacy_mode, ...) is nested under ``status``. A raw
        # device /status carries those fields at the top level. Support both.
        envelope = snap.get("status") if isinstance(snap.get("status"), dict) else snap
        details = envelope.get("details") or {}
        status = str(envelope.get("equipment_status") or "").strip()
        # Gateway-fronted cameras report `unknown` when unreachable (STATUS_SPEC §2.1).
        if status and status not in ("ready", "busy", "degraded"):
            return False, f"camera not ready (status: {status})", None
        if details.get("privacy_mode"):
            return False, "camera in privacy mode", None
        if details.get("streaming_enabled") is False:
            return False, "camera streaming disabled", None
        if details.get("go2rtc_reachable") is False:
            return False, "stream backend (go2rtc) down", None

        lens = self._pick_lens(details.get("lenses"))
        if lens is None:
            return False, "camera reports no lenses", None
        # Do NOT gate on lens.stream_connected. go2rtc connects to the camera
        # on demand — the producer stays idle (stream_connected False) until a
        # consumer subscribes, so blocking on it is a chicken-and-egg that
        # stops the preview from ever starting. Verified live: an MSE consumer
        # gets the codec init + a steady flow of segments even while
        # stream_connected reports False. The privacy/streaming/go2rtc checks
        # above are the real capability gates.
        mse_url = lens.get("mse_url")
        if not mse_url:
            return False, "camera lens has no stream url", None
        return True, None, self._absolute_ws(str(mse_url))

    def _find_camera_snapshot(self, data) -> Optional[Dict[str, Any]]:
        """Pull our camera's snapshot out of /api/equipment (list or dict)."""
        items = data
        if isinstance(data, dict):
            # Tolerate {"equipment": [...]}, {"devices": [...]}, or an id-keyed map.
            for key in ("equipment", "devices", "items"):
                if isinstance(data.get(key), list):
                    items = data[key]
                    break
            else:
                if self.camera_id in data and isinstance(data[self.camera_id], dict):
                    return data[self.camera_id]
                items = list(data.values())
        if not isinstance(items, list):
            return None
        for entry in items:
            # Raw device /status uses ``equipment_id``; the dashboard's
            # EquipmentSnapshot uses ``id``. Match either.
            if isinstance(entry, dict) and self.camera_id in (
                entry.get("equipment_id"), entry.get("id")
            ):
                return entry
        return None

    def _pick_lens(self, lenses) -> Optional[Dict[str, Any]]:
        if not isinstance(lenses, list) or not lenses:
            return None
        if self.lens:
            for lens in lenses:
                if isinstance(lens, dict) and str(lens.get("id")) == self.lens:
                    return lens
        first = lenses[0]
        return first if isinstance(first, dict) else None

    def _absolute_ws(self, mse_url: str) -> str:
        """Resolve a (possibly relative) mse_url to an absolute ws(s):// URL."""
        low = mse_url.lower()
        if low.startswith(("ws://", "wss://")):
            return mse_url
        if low.startswith("http://"):
            return "ws://" + mse_url[len("http://"):]
        if low.startswith("https://"):
            return "wss://" + mse_url[len("https://"):]
        # Relative path: anchor on the stream origin (or the dashboard API).
        base = self.stream_base_url or self.dashboard_base_url
        ws_base = base
        if base.lower().startswith("https://"):
            ws_base = "wss://" + base[len("https://"):]
        elif base.lower().startswith("http://"):
            ws_base = "ws://" + base[len("http://"):]
        if not mse_url.startswith("/"):
            mse_url = "/" + mse_url
        return ws_base.rstrip("/") + mse_url

    # ------------------------------------------------------------------
    # Resolution
    # ------------------------------------------------------------------

    def resolve_view(self, tags) -> Optional[str]:
        """Return the view name for a node's tags, or None if no station matches.

        The first tag (in ``stations`` config order) that maps to a view
        wins, so a ``[hood, shaker]`` node resolves via ``shaker``.
        """
        tag_set = set(tags or ())
        for tag, view in self._stations.items():
            if tag in tag_set:
                return view
        return None

    # ------------------------------------------------------------------
    # Entry point from the controller
    # ------------------------------------------------------------------

    def notify_node(self, node) -> None:
        """Point the camera at the station for ``node``. Never raises.

        ``node`` is duck-typed: anything exposing ``.tags`` (and optionally
        ``.id``) works, so the controller can pass a ``motion_graph.Node``
        and tests can pass a lightweight stand-in.
        """
        if not (self.configured and self.following) or node is None:
            return
        try:
            tags = getattr(node, "tags", ()) or ()
            view = self.resolve_view(tags)
            if view is None:
                return  # off-station node (safe/transit/home): leave camera as-is

            with self._lock:
                if view == self._last_view:
                    return  # dedupe: already looking there
                self._last_view = view

            body = self._views.get(view)
            action_and_payload = self._body_for_view(view, body)
            if action_and_payload is None:
                return
            action, payload = action_and_payload

            url = f"{self.dashboard_base_url}/api/equipment/{self.camera_id}/control/{action}"
            headers = {"Content-Type": "application/json"}
            api_key = self._environ.get(self._api_key_env) if self._api_key_env else None
            if api_key:
                headers["X-Api-Key"] = api_key

            node_id = getattr(node, "id", None)
            print(f"[camera] node {node_id!r} -> view {view!r} ({action})")
            self._sender(url, payload, headers, self._timeout_s)
        except Exception as exc:  # noqa: BLE001 - observability must not break motion
            print(f"[camera] notify_node failed (ignored): {exc}")

    def _body_for_view(
        self, view: str, body: Optional[Dict[str, Any]]
    ) -> Optional[tuple[str, Dict[str, Any]]]:
        """Map a view config to a (control-action, POST-body) pair."""
        if not body:
            print(f"[camera] view {view!r} has no config; skipping")
            return None
        preset_id = body.get("preset_id")
        if preset_id not in (None, "", "REPLACE_ME"):
            return "preset/goto", {"preset_id": preset_id}
        ptz = body.get("ptz")
        if isinstance(ptz, dict) and ptz:
            return "ptz", dict(ptz)
        print(f"[camera] view {view!r} has no usable preset_id or ptz body; skipping")
        return None

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    def _threaded_send(
        self, url: str, payload: Dict[str, Any], headers: Dict[str, str], timeout_s: float
    ) -> None:
        threading.Thread(
            target=self._http_post,
            args=(url, payload, headers, timeout_s),
            name="xarm-camera-tracker",
            daemon=True,
        ).start()

    @staticmethod
    def _http_post(
        url: str, payload: Dict[str, Any], headers: Dict[str, str], timeout_s: float
    ) -> None:
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(url, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=timeout_s):
                pass
        except Exception as exc:  # noqa: BLE001 - best-effort by contract
            print(f"[camera] POST {url} failed (ignored): {exc}")

    @staticmethod
    def _http_get_json(url: str, timeout_s: float) -> Any:
        request = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read().decode("utf-8"))
