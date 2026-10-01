# Rotating the xArm 180° on the rail — plan and checklist

Status: **planned** (written 2026-09-24, not yet executed). Nothing in this
document has been applied to the code or the configuration yet; sections
2 and 3 list the software that still has to be written before move day.

Run the checklist in order. Each block has a **PASS** gate: if it fails,
stop, and fix or skip with a note rather than carrying on.

---

## 1. What changes, and why it is a rotation (not a mirror)

The robot is turned 180° about its own base axis (joint 1's vertical axis).
The stations do not move in the room, but the robot's own frame now points
the other way.

- A 180° turn flips **both** horizontal robot axes at once: the robot's +X
  becomes the room's old −X, and its +Y becomes the old −Y. Z is unchanged.
- That double flip is why the rail *looks* reversed from the robot's point
  of view. It is not a mirror: a mirror flips one axis and would put every
  station on the wrong side. You cannot mirror a rigid robot by moving it.

| Item | Where it lives | Change |
|---|---|---|
| 51 arm poses | `src/settings/joint_config.yaml` (joint angles) | **J1 −180° on every pose**, J2–J5 unchanged. Same arm shape in the room, gripper orientation included. |
| Rail locations | `src/settings/linear_track_config.yaml` (`Home 0`, `Deck 550`, `Cytation 700`, …) | **Case A** (rail untouched, home end stays at the same end of the room): unchanged. **Case B** (whole rail + robot turned, home end at the other end): `new = c − old`, with `c` measured (likely ≈ 700, but measure it). |
| Graph nodes, edges, gripper states | `src/settings/motion_graph.yaml` | Unchanged: nodes reference poses and rail locations by name. |
| Hood danger volume | `src/settings/interlocks.yaml` (`x_min: 200`, robot frame) | Flips to `x < −200`. Not enforced today; needs an "x less than" field in code. |
| Workspace limits | `src/settings/safety.yaml` (±700 mm in x and y) | Unchanged: symmetric. |
| Direct Drive jog buttons | `src/web/main.js` `jog()` / `jogMap` | Rotate each button's direction by the mount angle, so **X+ / Y+ still move the arm the way the button shows in the room**. At 180°: X+ sends dx −1, Y+ sends dy −1. Z unchanged. |
| Arm-controller settings | UFACTORY Studio (safety fence, reduced-mode boundary) | Stored in the robot frame. Re-check by hand. |

### Why the same −180° on every pose

After the shift J1 runs from −226° to 90° (today: −46° to 270°), inside the
xArm5's ±360° J1 limit. Do **not** mix +180° and −180° to keep angles
small. Moves interpolate joint angles, so if an edge's two ends get
different offsets, the arm swings the other way round between them. For
example, `robot_home` (180°) → `opentrons_home` (−4°) sweeps through 90°
today; with mixed offsets it would sweep through the opposite side, a
likely collision. A uniform offset keeps every swing identical to today's.

### Why the panel must change

Operators read the jog buttons in room directions ("X+ is towards the
deck"). After the turn the robot frame is reversed, so an unchanged panel
would move the arm opposite to the button, which is confusing and unsafe
near equipment. The agent API stays in the robot frame; only the panel
maps room directions to robot directions.

---

## 2. Open decision (answer before move day)

- [ ] **Case A or Case B?** Does the rail's home end (its home switch / motor
  end) stay at the same end of the room?
  - **A — yes:** only arm poses and the panel change.
  - **B — no:** also measure `c`: after the move, home the rail, drive to one
    station by hand and read the rail position there. `c = measured + old`
    for that station (e.g. if Deck, old 550, now reads 150, then c = 700).

---

## 3. Software to build before move day (reversible; changes nothing until enabled)

- [ ] **Mount-angle setting.** `base_yaw_deg` on the `robot` profile in
  `src/settings/xarm_config.yaml`, default `0`. Reported in `/status`
  (`details.connection_details`). At `0` nothing changes.
- [ ] **Panel jog mapping.** `jog()` in `src/web/main.js` rotates the
  button's `(dx, dy)` by `base_yaw_deg` before POSTing
  `/control/freehand/relative`. Bump the `main.js?v=` cache-buster.
- [ ] **Pose transform script** `tools/rotate_base.py`:
  - J1 −180° on every pose in `joint_config.yaml`, uniformly;
  - edits the text in place so comments survive (no YAML re-dump);
  - refuses to write if any J1 would leave ±360°;
  - Case B: `--rail-offset c` rewrites `linear_track_config.yaml` locations
    to `c − old` and refuses values outside 0–700;
  - prints the full before/after diff; writes only with `--apply`.
    Git is the backup.
- [ ] **Hood volume.** Add `x_max` support to the danger volume so
  `x_min: 200` can become `x_max: -200`.
- [ ] **Tests.** Script (uniform offset, limit refusal, comments kept, Case B
  rail maths) and jog mapping at 0° and 180°.
- [ ] **Docs.** Note in `src/docs/agent/API_REFERENCE.md` that freehand
  coordinates are robot-frame and unaffected by `base_yaw_deg`.

**PASS:** full test suite green; deployed with `base_yaw_deg: 0` and the
panel behaves exactly as before.

---

## 4. Before the move — record a reference

- [ ] Graph STRICT, safety level Low, E-stop in reach.
- [ ] Park at each of: `robot_home`, `opentrons_home`, `deck_home`,
  `uplc_home`, and one plate pose such as `opentrons_2_low`. At each, record:
  - rail position (`metrics.track_position` on `/status`);
  - TCP pose (`details.current_position`) and joints (`details.current_joints`);
  - a RealSense capture labelled with the node, e.g.
    `POST /control/realsense/capture {"camera": "rs405", "label": "pre-rotation", "node_id": "<node>", "protected": true}`.
- [ ] Note the git commit deployed on the PC (`git log --oneline -1`).

**Collect:** the table of readings above; capture ids.

---

## 5. Move day

- [ ] Release claims, **Disconnect**, power the arm down.
- [ ] Remount the robot turned 180°. The base axis must land on the **same
  spot of the carriage**. If it moves even a few mm, plan to re-teach the
  precise poses (the plate "low/press" poses are only ~4 mm apart).
- [ ] Check cable routing and slack through the full new J1 range
  (−226° to 90°).
- [ ] Case B only: home the rail and measure `c` (section 2).
- [ ] On the PC checkout:
  ```powershell
  cd C:\Users\sdl2\Projects\xarm-translocation
  .venv\Scripts\python.exe tools\rotate_base.py                 # dry run: read the diff
  .venv\Scripts\python.exe tools\rotate_base.py --apply         # Case A
  .venv\Scripts\python.exe tools\rotate_base.py --apply --rail-offset <c>   # Case B
  ```
- [ ] Set `base_yaw_deg: 180` on the `robot` profile; flip the hood volume.
- [ ] Commit the config change, then `C:\SDL_Tools\nssm.exe restart xarm`.
- [ ] In UFACTORY Studio: re-check the safety fence / reduced-mode boundary
  and any TCP or payload settings.

**PASS:** service healthy; `/status` shows `base_yaw_deg: 180`; the diff
showed only J1 changes (plus rail values in Case B).

---

## 6. Verification (graph STRICT, safety level Low, hand on the E-stop)

- [ ] **Connect** with the gripper free of contact (the F/T sensor zeroes on
  connect).
- [ ] **Jog directions.** Enforcement must be OFF for freehand, so an
  administrator turns it OFF for this step only. With step 1 mm: X+, X−,
  Y+, Y−, Z+, Z− each move the way the button shows in the room. Turn
  enforcement back ON afterwards.
- [ ] **Recover** to `robot_home` (Recover to Node).
- [ ] **Walk the graph slowly.** Travel every edge at low speed, starting
  from `robot_home`, and stop at each reference node from section 4.
  Compare rail, joints and the RealSense capture with the reference.
- [ ] **Re-teach** any pose that is off by more than ~1–2 mm, using
  `POST /control/graph/pose` with `overwrite: true` and a comment.
- [ ] Run one real plate transfer end to end at low speed.

**PASS:** every edge driven without contact; reference nodes match within
~1–2 mm; jog directions match the buttons.

**Collect on failure:** node, edge, expected vs. measured joints/rail, a
RealSense capture, and `C:\SDL_Logs\xarm.out.log` around the time.

---

## 7. Rollback

- Revert the configuration commit (`joint_config.yaml`, and
  `linear_track_config.yaml` in Case B), set `base_yaw_deg: 0`, restart the
  service, and remount the robot in its original orientation.
- Nothing in this plan is irreversible in software; the physical remount is
  the only step that takes real time to undo.
