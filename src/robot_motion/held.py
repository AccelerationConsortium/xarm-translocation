"""Whether the gripper holds a part: one record for routines, Graph Drive and
freehand gripper commands.

A caged vial (the Gibbie fingertips) is never felt by the Robotiq, so
"holding" cannot be read off the gripper: it is set when a command could have
taken a part (a close to the grip stroke or further, a grasp catalog state, or
the fingers stopping on contact) and cleared only where letting go is safe (a
routine's put-down, a release at a node that allows one) or by a person who
looked (POST /gripper/confirm_empty). It is kept in a small JSON file beside
the audit file so a restart remembers it; with no readable file the gripper
counts as holding until a person says otherwise (fail closed).

Only graphs with `held_part_guard: true` use it; elsewhere the gripper keeps
its previous, unguarded behaviour.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path

log = logging.getLogger(__name__)

SOURCES = ("routine", "graph", "freehand", "attestation", "unknown")
CONTACT = ("contact_closing", "contact_opening")


def _now():
    return datetime.now(timezone.utc).isoformat()


def _who(identity):
    return identity["email"] if identity else None


class HeldPart:
    def __init__(self, path: str | os.PathLike | None):
        self.path = Path(path) if path else None
        self.state = self._load()

    @property
    def held(self) -> bool:
        return bool(self.state["held"])

    def summary(self):
        return dict(self.state)

    def set(self, source, identity, raw):
        if source not in SOURCES:
            raise ValueError(source)
        if self.held and self.state["source"] != "unknown":
            return
        self.state = {
            "held": True, "source": source, "since": _now(), "set_by": _who(identity),
            "raw": raw, "cleared_by": None, "cleared_at": None, "reason": None,
        }
        self._save()

    def clear(self, source, identity, raw):
        if source not in SOURCES:
            raise ValueError(source)
        if not self.held:
            return
        self.state = {
            **self.state, "held": False, "source": source, "raw": raw,
            "cleared_by": _who(identity), "cleared_at": _now(), "reason": None,
        }
        self._save()

    def _unknown(self, reason):
        return {
            "held": True, "source": "unknown", "since": _now(), "set_by": None, "raw": None,
            "cleared_by": None, "cleared_at": None, "reason": reason,
        }

    def _load(self):
        if self.path is None:
            return self._unknown("No saved state (no audit file configured)")
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return self._unknown("No saved state (first start)")
        except (OSError, ValueError) as exc:
            log.warning("held-part state unreadable, treating the gripper as holding: %s", exc)
            return self._unknown(f"Saved state unreadable: {exc}")
        if not isinstance(data, dict) or not isinstance(data.get("held"), bool) or data.get("source") not in SOURCES:
            return self._unknown("Saved state malformed")
        return {**self._unknown(None), **data}

    def _save(self):
        if self.path is None:
            return
        tmp = self.path.with_name(self.path.name + ".tmp")
        try:
            tmp.write_text(json.dumps(self.state), encoding="utf-8")
            os.replace(tmp, self.path)
        except OSError as exc:  # the in-memory state still guards this run
            log.error("held-part state not saved: %s", exc)
