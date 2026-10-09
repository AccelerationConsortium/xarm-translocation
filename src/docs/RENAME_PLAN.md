# Renaming the repo to Armiad — plan and checklist

Status: **planned** (name chosen 2026-10-08, not yet done). `ur-robot`
(package `robot-motion`) was merged into `main` on 2026-10-08 (f638bce);
do the rename at the next release. Nothing in this document has been
applied yet.

---

## 1. Why rename, and why Armiad

`xarm-translocation` no longer describes the repo. The `ur-robot` branch
drives a UR5e as well as the xArm, MG400 is planned, and the shared panel
is already titled "Robot Motion". The name should not carry a robot brand.

**Armiad** = *arm* + *Ariad(ne)*. In the myth, Ariadne gave Theseus a ball
of thread to find his way through the Labyrinth. The motion graph is that
thread: teach the arm its poses and paths once, and it always finds its
way between stations along routes that were checked.

README opening line:

> Armiad: like Ariadne's thread, for robot arms. Teach the paths once,
> and the arm always finds its way.

GitHub About description:

> Teach a robot arm its poses once, link them into a motion graph, then
> send it anywhere with one click. Web panel and REST API for xArm and UR
> arms.

Topics: `motion-graph`, `robot-arm`, `lab-automation`, `self-driving-lab`,
`xarm`, `universal-robots`.

### Availability (checked 2026-10-08)

`armiad` is free in the AccelerationConsortium GitHub organisation and on
PyPI, and no notable GitHub project uses it. Check again before renaming.

### Names considered and dropped

| Name | Why not |
|---|---|
| `robot-motion` | Free, and matches the package, but plain. Kept as the fallback. |
| `motion-graph`, `movegraph` | Free. Descriptive, but less memorable. |
| `ariadne` | Taken on PyPI by a popular GraphQL library; `import ariadne` would clash. |
| `ariad` | Free, but has no arm or robot reference. |
| `armadne`, `ariabot`, `armthread` | Free. Harder to say, or a weaker echo of the myth. |
| `talos`, `golem`, `atlas`, `janus`, `theseus` | Taken or crowded (Talos Linux, Boston Dynamics Atlas, JanusGraph, Meta's Theseus). |
| `clew` | The actual word for Ariadne's ball of thread, but too obscure. |

---

## 2. Decisions still open

1. **Python package and CLI name.** Either keep `robot-motion` (no change
   on device PCs or in `lab_skills`) or rename it to `armiad` as well, so
   it is one name everywhere. `armiad` is free on PyPI.
2. **Service names on device PCs** (`xarm`, `robot-motion-prototype`). The
   recommendation is to leave them: a service rename means re-registering
   NSSM, and is not needed for the repo rename.

---

## 3. Checklist

### 3.1 GitHub

- [ ] Settings → Repository name → `armiad`. GitHub redirects the old
      web, clone and issue URLs. **Never create a new repo named
      `xarm-translocation` afterwards**: that breaks the redirect.
- [ ] Set the About description and topics from section 1.

### 3.2 In this repo

- [ ] `pyproject.toml` `[project.urls]`: point all four links at
      `https://github.com/AccelerationConsortium/armiad` (they use
      `xarm-translocation` since the merge).
- [ ] README title and opening paragraph, with the tagline from section 1.
- [ ] Docstrings that name the repo: `src/core/models.py`,
      `test/__init__.py`, `test/conftest.py`.
- [ ] `src/core/assistant_llm.py` mentions the
      `C:\Users\sdl2\Projects\xarm-translocation\.env` path on sdl2-pc-03.
      Only change it if that folder is renamed (see 3.4).
- [ ] A CHANGELOG entry for the rename.

### 3.3 Clones and remotes

- [ ] This Linux machine: `git remote set-url origin
      https://github.com/AccelerationConsortium/armiad.git` in
      `~/caoyang/xarm-translocation`. The `~/caoyang/robot-motion`
      worktree shares that `.git`, so one change covers both.
- [ ] Device PCs: the same `set-url` in each checkout (sdl2-pc-03 for the
      xArm on `debug-xarm5`, sdl2-pc-05 for the UR5e on `debug-ur5e`).
      Branch names don't change.

### 3.4 Device PC folders (leave them unless there is a reason)

The checkouts live at `C:\Users\sdl2\Projects\xarm-translocation` (xArm)
and `C:\Users\sdl2\Projects\robot-motion` (UR5e). NSSM services point at
these folders and `.env` / `.state` live inside them, so the repo rename
does **not** require moving them. If a folder is renamed later, do it as
a separate scheduled change: Disconnect the arm first, stop the service,
move the folder, re-point the NSSM `AppDirectory` and paths, then start.

### 3.5 Other repos that name `xarm-translocation`

Found by grep on 2026-10-08. Update live docs and code; leave dated
historical records as they are.

- `ac-organic-lab`: `locations.yaml`,
  `skills/src/lab_skills/typed_clients/robot_arm.py`,
  `skills/src/lab_skills/status_adapters/legacy.py`,
  `skills/tests/test_status_adapters.py`, and docs `AUTH_DESIGN.md`,
  `DEVICE_PC_SETUP.md`, `LAB_MONITORING.md`, `UI_DESIGN.md`,
  `PLATE_TRACKING.md`, `ROADMAP.md`. The roadmap's commit history is
  historical; leave it.
- `sdl-camera-server`: `README.md` (the "camera code extracted from
  xarm-translocation" note).
- `ops/records/VERIFICATION-2026-09-17.md`: historical, leave.

### 3.6 After the rename

- [ ] `git fetch` works on every clone with the new remote.
- [ ] Re-run the grep for `xarm-translocation` across the lab repos;
      only historical records should remain.
