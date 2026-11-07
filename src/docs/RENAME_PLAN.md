# Renaming the repo to robot-motion — plan and checklist

Status: **decided, not yet done.** On 2026-10-28 the name became
`robot-motion`, replacing Armiad (chosen 2026-10-08, dropped because it
sounded strange). Nothing in the checklist below has been applied yet.

---

## 1. Why rename, and why robot-motion

`xarm-translocation` no longer describes the repo. It drives the UFactory
xArm5 and the Universal Robots UR5e, UR3e and UR5 CB3, with Dobot MG400
planned, and the name should not carry one robot's brand.

`robot-motion` is the name already in use almost everywhere: the Python
package, the `robot-motion` command, the shared panel's "Robot Motion"
title, and the checkout folder on the three UR device PCs. Renaming the
repo makes it one name everywhere.

GitHub About description (applied 2026-10-28, before the rename):

> Motion-graph control for laboratory robot arms: web panel and REST API
> for UFactory xArm5 and Universal Robots UR5e, UR3e and UR5 CB3, with
> Dobot MG400 planned

Topics (applied 2026-10-28): `robot-arm`, `motion-graph`,
`lab-automation`, `self-driving-lab`, `xarm`, `universal-robots`.

### Names considered and dropped

| Name | Why not |
|---|---|
| `armiad` | Chosen 2026-10-08 (*arm* + *Ariadne*), dropped 2026-10-28: it sounded strange. |
| `motion-graph`, `movegraph` | Descriptive, but differ from the package name. |
| `ariadne` | Taken on PyPI by a popular GraphQL library; `import ariadne` would clash. |
| `ariad`, `armadne`, `ariabot`, `armthread`, `clew` | No arm reference, hard to say, or too obscure. |
| `talos`, `golem`, `atlas`, `janus`, `theseus` | Taken or crowded (Talos Linux, Boston Dynamics Atlas, JanusGraph, Meta's Theseus). |

Check that `robot-motion` is still free in the AccelerationConsortium
organisation before renaming.

### What the rename does not change

- **The lab equipment id `xarm_translocation`** (underscore) in
  `ac-organic-lab`'s `equipment.yaml`, `locations.yaml` and status
  adapters. It names the xArm in the dashboard, claims, location aliases
  and records, not the repo. Leave it.
- **Deployments.** Device PCs run their checked-out branch and get updates
  by git bundle, so no running service reads the repo URL. Folders, NSSM
  services and `.env` / `.state` paths stay where they are.

## 2. Decisions

1. **Python package and CLI name:** already `robot-motion`; no change.
2. **Service names on device PCs** (`xarm`, `robot-motion-prototype`,
   `robot-motion-ur3e`, `robot-motion-ur5cb3`): leave them. A service
   rename means re-registering NSSM, and the repo rename does not need it.

---

## 3. Checklist

### 3.1 GitHub

- [ ] Settings → Repository name → `robot-motion`. GitHub redirects the old
      web, clone and issue URLs. **Never create a new repo named
      `xarm-translocation` afterwards**: that breaks the redirect.
- [x] Set the About description and topics from section 1 (2026-10-28).

### 3.2 In this repo

- [ ] `pyproject.toml` `[project.urls]`: point all four links at
      `https://github.com/AccelerationConsortium/robot-motion` (they use
      `xarm-translocation` since the merge).
- [ ] Docstrings that name the repo: `src/core/models.py`,
      `test/__init__.py`, `test/conftest.py`.
- [ ] `src/core/assistant_llm.py` mentions the
      `C:\Users\sdl2\Projects\xarm-translocation\.env` path on sdl2-pc-03.
      Only change it if that folder is renamed (see 3.4).
- [ ] A CHANGELOG entry for the rename.

### 3.3 Clones and remotes

- [ ] This Linux machine: `git remote set-url origin
      https://github.com/AccelerationConsortium/robot-motion.git` in
      `~/caoyang/xarm-translocation`. The `~/caoyang/robot-motion`
      worktree shares that `.git`, so one change covers both.
- [ ] Device PCs: the same `set-url` in each checkout, if it has a
      remote at all (they have no git credentials and update by bundle):
      sdl2-pc-03 (xArm5, `debug-xarm5`), sdl2-pc-05 (UR5e, `debug-ur5e`),
      sdl2-pc-04 (UR3e, `debug-ur3e`) and sdl2-pc-00-lle (UR5 CB3,
      `debug-ur5cb3`). Branch names don't change.

### 3.4 Device PC folders (leave them unless there is a reason)

The checkouts live at `C:\Users\sdl2\Projects\xarm-translocation` (xArm)
and `C:\Users\sdl2\Projects\robot-motion` (the three UR PCs, which
already carry the new name). NSSM services point at
these folders and `.env` / `.state` live inside them, so the repo rename
does **not** require moving them. If a folder is renamed later, do it as
a separate scheduled change: Disconnect the arm first, stop the service,
move the folder, re-point the NSSM `AppDirectory` and paths, then start.

### 3.5 Other repos that name `xarm-translocation`

Found by grep on 2026-10-08. Update live docs and code; leave dated
historical records as they are.

- `ac-organic-lab`: comments and docstrings only, never the
  `xarm_translocation` equipment id (section 1). `locations.yaml`,
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
