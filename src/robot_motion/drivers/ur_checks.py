"""Controller safety queries that tell a refusal from a query that did not complete.

ur_rtde's isPoseWithinSafetyLimits, isJointsWithinSafetyLimits and
getInverseKinematicsHasSolution return False both when the controller says
no and when the command never completed (sendCommand failed: the control
script not running, a timeout). A False alone is therefore not evidence that
a pose is out of limits.

check() asks with the link verified before and after, and on a False asks
the same question about the arm's current state (its joints or TCP pose),
which a controller running with safety NORMAL accepts. Only a True for that
probe on the same link makes the False a confirmed refusal. None of this
moves, reconnects or resets anything; with the control script stopped
nothing is asked at all, because ur_rtde would re-upload its script to
answer.

Results: passed, rejected (confirmed), query_failed (did not complete),
indeterminate (a False the probe could not confirm), not_checked (not asked).
"""

from __future__ import annotations

PASSED = "passed"
REJECTED = "rejected"
QUERY_FAILED = "query_failed"
INDETERMINATE = "indeterminate"
NOT_CHECKED = "not_checked"
RESULTS = (PASSED, REJECTED, QUERY_FAILED, INDETERMINATE, NOT_CHECKED)
# What the controller says about which limit refused: nothing, for these
# queries. Reported as such rather than guessed.
BOOLEAN_ONLY = {
    "known": False,
    "note": "ur_rtde answers this query with a boolean only; the controller does not say which "
    "joint, safety plane or tool-orientation limit refused",
}


def link_problem(control):
    """Why the control link cannot answer a query now, or None."""
    try:
        if not control.isConnected():
            return "control_link_down"
        if control.isProgramRunning() is not True:
            return "control_script_stopped"
    except Exception as exc:  # noqa: BLE001 - reported as the reason
        return f"link_check_failed: {exc}"
    return None


def ask(control, query, *args):
    """One controller query with the link checked before and after.

    {query, sent, completed, answer, error}: completed only when the call
    returned and the link and script were still up afterwards.
    """
    problem = link_problem(control)
    if problem is not None:
        return {"query": query, "sent": False, "completed": False, "answer": None, "error": problem}
    try:
        answer = getattr(control, query)(*args)
    except Exception as exc:  # noqa: BLE001 - a failed query is a result
        return {"query": query, "sent": True, "completed": False, "answer": None, "error": str(exc)}
    problem = link_problem(control)
    if problem is not None:
        return {"query": query, "sent": True, "completed": False, "answer": answer,
                "error": f"{problem} after the query"}
    return {"query": query, "sent": True, "completed": True, "answer": answer, "error": None}


def check(control, query, args, probe_args, probe_of):
    """Classify a boolean safety query (see the module docstring).

    probe_args: the same query about the arm's current state, described by
    probe_of ("current joints", "current TCP pose").
    """
    first = ask(control, query, *args)
    record = {"query": query, "completed": first["completed"], "answer": first["answer"]}
    if not first["completed"]:
        record["result"] = NOT_CHECKED if not first["sent"] else QUERY_FAILED
        record["reason"] = first["error"]
        return record
    if first["answer"] is True:
        record["result"] = PASSED
        return record
    if first["answer"] is not False:
        record["result"] = QUERY_FAILED
        record["reason"] = f"unexpected answer {first['answer']!r}"
        return record
    probe = ask(control, query, *probe_args)
    record["probe"] = {"of": probe_of, "completed": probe["completed"], "answer": probe["answer"],
                       "error": probe["error"]}
    if probe["completed"] and probe["answer"] is True:
        record["result"] = REJECTED
        record["reason"] = f"the controller answered false here and true for the {probe_of} on the same link"
    else:
        record["result"] = INDETERMINATE
        record["reason"] = (
            f"the same query about the {probe_of} did not answer true either, so this false "
            "cannot be told from a query that did not complete"
        )
    return record


def recheck(control, query, args, probe_args, probe_of, first_answer):
    """check() after a fast raw query answered first_answer (not True)."""
    record = check(control, query, args, probe_args, probe_of)
    record["first_answer"] = first_answer
    if record["result"] == PASSED:
        record["result"] = INDETERMINATE
        record["reason"] = f"answered {first_answer!r}, then true when asked again"
    return record
