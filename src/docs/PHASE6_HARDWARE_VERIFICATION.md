# Phase 6 — Hardware Verification Checklist

Verifies that motion-graph phases 1–5 actually do what the unit tests
promise, against the real xArm5 + UPLC drawer setup.

**You drive; I sit on standby.** Run each section in order; the later
sections rely on state that earlier ones set up. Each block has a
PASS gate — if it fails, stop and either fix or skip with a note;
don't paper over an unexpected result.

For every block: the **expected** outcome is the success condition;
the **collect** lines are the artifacts I'll want to see if you ask
me to diagnose a failure.

---

## 0. Pre-flight (do this once)

- [ ] **Workspace is clear** of fragile glassware in the arm's
  reachable envelope.
- [ ] **E-stop is physically accessible** from the keyboard position.
- [ ] **Latest code is deployed**: pull on this PC, then
  `nssm restart xarm` (PowerShell, elevated). Verify with:
  ```powershell
  curl http://localhost:8000/ | Select-String protocol_version
  # expected: "protocol_version": "1.1"
  ```
- [ ] **Browser**: open http://localhost:8000/web/ and clear cache for
  the page (Ctrl+F5) so the new `main.js?v=…` cache-buster loads.
- [ ] **Logs**: open a PowerShell tail on the service stdout so you
  can see `[motion_graph]` advisory messages:
  ```powershell
  Get-Content C:\SDL_Logs\xarm.out.log -Tail 20 -Wait
  ```

If something looks wrong at this stage, fix it before moving on.

---

## 1. Phase 1 — observation only (ADVISORY)

Goal: prove `current_node` tracks named moves and clears on raw moves.

1. **Connect** via the web UI with profile `robot`. Wait for `ready`.
2. Open the Motion Graph card. **Expected**: visible, `Mode: advisory`,
   `Current: (off-grid)`, `Reachable: (off-grid — use Recover)`.
3. From the named-locations dropdown, **Move to `robot_home`**.
   Expected: arm moves. After arrival, Motion Graph card updates —
   `Current: home` and `Reachable` shows one or two buttons depending
   on which edges from `home` are in the YAML.
4. **Move to `uplc_draw_home`** (the named preset, not the node id).
   Expected: arm moves. `Current` updates to `uplc_draw_approach` (the
   node id is different from the preset name; that's intentional).
5. **Run a raw Cartesian move** (use the "Move to Position" form with
   any nearby safe coordinates). Expected: `Current: (off-grid)`,
   Reachable shows the off-grid message. Logs should show no error.
6. **Stop** mid-move (start a long named move, hit STOP). Expected:
   arm halts, `Current` clears to `(off-grid)`. Service still responds
   to `/status`. Click **Clear Errors**, then **Enable** to recover.

**PASS gate**: steps 3–6 all behave as expected.
**Collect on failure**: `/status` response + log tail for the failing step.

---

## 2. Phase 2 — STRICT mode + edge dispatch

Goal: prove off-whitelist moves are refused with 409 and edge.mode /
edge.speed actually steer dispatch.

1. **Move to `robot_home`** to re-pin (the previous section ended off-grid).
2. **Switch Mode → `strict`** via the dropdown. Expected: 200, no
   visible behavior change, and `Reachable` now lists only the edges
   the YAML actually contains from `home`.
3. **Move to a reachable node** (one of the buttons). Expected: 200,
   arm moves, `Current` updates to that node.
4. **Try an off-whitelist named move**. Easiest: pick a position in
   the dropdown that has no edge from the current node (e.g. jump
   straight to `uplc_plate_in` while at `home`).
   Expected: HTTP 409 with body
   `{error: "edge_not_allowed", current_node, target, reason}`. Arm
   does **not** move. Log entry in red.
5. **Verify edge.mode override**: in the YAML, the `home →
   uplc_draw_approach` edge is `mode: joint`. The preset `uplc_draw_home`
   is a joint-list, so this is the natural-fit case. Move there from
   `home` and confirm the arm moves in joint mode (smooth coordinated
   joint motion, not Cartesian-straight).
6. **Verify edge.speed cap**: in the YAML, `uplc_draw_approach →
   uplc_draw_open_max` is `speed: 20`. Issue the move with a higher
   speed override (e.g. `curl -X POST /control/graph/move_to -d
   '{"node_id":"uplc_draw_open_max","speed":80}'`). Expected: the
   arm moves at ~20, not 80. Service log shows
   `[motion_graph] clamping speed 80 -> 20`.
7. **Record an edge**: drive a transition that doesn't have an edge
   yet (in advisory mode briefly: switch Mode → `advisory`, make the
   move, switch back to `strict`). Then
   `curl -X POST http://localhost:8000/control/graph/record
   -H "Content-Type: application/json" -d '{"comment": "verified
   2026-05-24"}'`. Expected: 200 with body listing the appended
   edge. Open `src/settings/motion_graph.yaml` — verify the new edge
   is at the end and comments above are intact.

**PASS gate**: 4 and 6 are the critical ones (rejection + speed cap).
**Collect on failure**: the 409 response body, or the actual move
duration / log line for the speed-cap check.

After this section, switch Mode back to `advisory` for the rest of
verification so you're not fighting strict-mode rejections in the
remaining sections.

---

## 3. Phase 3 — v1.1 contract + claim protocol

Goal: prove `protocol_version: "1.1"`, claim acquire/heartbeat/release
work, and `details.claimed_by` + `allowed_actions` populate correctly.

Run these from a separate PowerShell so the web session is unaffected:

1. **Acquire a claim**:
   ```powershell
   $r = Invoke-RestMethod -Method Post http://localhost:8000/control/claim `
     -ContentType "application/json" `
     -Body '{"owner": "verify-script", "session_id": "phase6-001"}'
   $r
   # expected: claim_token, heartbeat_interval_s, expires_at
   $tok = $r.claim_token
   ```
2. **Status carries claimed_by**:
   ```powershell
   curl http://localhost:8000/status | python -c "import sys,json; d=json.load(sys.stdin); print(d['details']['claimed_by'])"
   # expected: {session_id: phase6-001, owner: verify-script, expires_at: ...}
   ```
3. **Heartbeat extends TTL**:
   ```powershell
   $h = Invoke-WebRequest -Method Post http://localhost:8000/control/heartbeat `
     -Headers @{ "X-Claim-Token" = $tok }
   $h.StatusCode
   # expected: 204
   ```
4. **Conflict on second acquire**:
   ```powershell
   try { Invoke-RestMethod -Method Post http://localhost:8000/control/claim `
     -ContentType "application/json" `
     -Body '{"owner": "thief", "session_id": "phase6-002"}'
   } catch { $_.Exception.Response.StatusCode; $_.ErrorDetails.Message }
   # expected: Conflict (409), body lists claimed_by + retry_after_s
   ```
5. **Release is idempotent**:
   ```powershell
   $r = Invoke-WebRequest -Method Post http://localhost:8000/control/release `
     -Headers @{ "X-Claim-Token" = $tok }
   $r.StatusCode    # expected: 204
   # second release with same token also 204
   $r = Invoke-WebRequest -Method Post http://localhost:8000/control/release `
     -Headers @{ "X-Claim-Token" = $tok }
   $r.StatusCode    # expected: 204
   ```
6. **allowed_actions populates only in STRICT mode**:
   - In advisory: `curl /status | findstr allowed_actions` shows
     `["stop"]` (no `move.*` entries).
   - Switch to strict, move to a known node, re-fetch: now lists
     `["stop", "move.<reachable_node>", ...]`.

**PASS gate**: 1–5 all produce expected status codes; 6 shows the
mode-dependent population.

---

## 4. Phase 4 — off-grid recovery (no movement)

Goal: prove the nearest-node detector finds the right pose and
recover_to refuses on mismatch.

1. **Get off-grid**: do a small raw Cartesian move (5mm in any safe
   direction) so `current_node` clears. Confirm via `/status`.
2. **Detector suggests the node you're near**:
   ```powershell
   curl http://localhost:8000/graph/nearest
   # expected: suggested_node = the node you were at before the raw move,
   # arm_residual_deg small (under 10), within_tolerance: true
   ```
3. **Recover_to with mismatch is refused**:
   ```powershell
   # ask to recover to a DIFFERENT node than the suggested one
   try { Invoke-RestMethod -Method Post http://localhost:8000/control/graph/recover_to `
     -ContentType "application/json" `
     -Body '{"node_id": "<some_other_node>"}'
   } catch { $_.Exception.Response.StatusCode; $_.ErrorDetails.Message }
   # expected: UnprocessableEntity (422), body has recovery_mismatch + suggested
   ```
4. **Recover_to to the suggested node succeeds**:
   - Web UI: click **Recover…**. Expected: panel opens, "Nearest" shows
     the right node, force checkbox unchecked because within_tolerance.
     Click **Snap to suggested**. `Current` updates immediately.
   - The arm does NOT physically move — recover is a *declaration*.
5. **force=true bypass**:
   - Pick any reachable known node and the detector likely doesn't
     match it. Recover_to that node with `force=true`. Expected: 200,
     `Current` set to that node. (You'd typically only do this for
     cartesian-dict presets the algo can't score.)

**PASS gate**: 2 finds the right node; 3 refuses the wrong one; 4 + 5
both update `current_node` without moving the arm.

---

## 5. Phase 5 — enforcement on /move/* and /gripper/*

Goal: prove `X-Claim-Token` is required when enforcement is on AND a
claim is held, and that safety endpoints are never gated.

1. **Acquire a claim** (use the PowerShell session from §3):
   ```powershell
   $r = Invoke-RestMethod -Method Post http://localhost:8000/control/claim `
     -ContentType "application/json" `
     -Body '{"owner": "verify", "session_id": "phase6-enforce"}'
   $tok = $r.claim_token
   ```
2. **Turn enforcement on** (this requires the claim since enforcement
   is currently OFF — but enabling from off is open per the bootstrap
   rule, so no token needed yet):
   ```powershell
   Invoke-RestMethod -Method Post http://localhost:8000/control/claim/enforce `
     -ContentType "application/json" `
     -Body '{"enabled": true}'
   # expected: {"enforced": true}
   ```
3. **Move without token is refused**:
   ```powershell
   try { Invoke-RestMethod -Method Post http://localhost:8000/move/location `
     -ContentType "application/json" `
     -Body '{"location_name": "robot_home"}'
   } catch { $_.Exception.Response.StatusCode }
   # expected: Locked (423)
   ```
4. **Move with the holder's token succeeds**:
   ```powershell
   Invoke-RestMethod -Method Post http://localhost:8000/move/location `
     -ContentType "application/json" -Headers @{ "X-Claim-Token" = $tok } `
     -Body '{"location_name": "robot_home"}'
   # expected: 200
   ```
5. **STOP is never blocked** (safety floor):
   ```powershell
   Invoke-RestMethod -Method Post http://localhost:8000/move/stop
   # expected: 200, no token required
   ```
6. **Clear errors never blocked**:
   ```powershell
   Invoke-RestMethod -Method Post http://localhost:8000/clear/errors
   # expected: 200
   ```
7. **Tear down**: turn enforcement off (requires the holder's token
   now that it's on), then release the claim:
   ```powershell
   Invoke-RestMethod -Method Post http://localhost:8000/control/claim/enforce `
     -ContentType "application/json" -Headers @{ "X-Claim-Token" = $tok } `
     -Body '{"enabled": false}'
   Invoke-WebRequest -Method Post http://localhost:8000/control/release `
     -Headers @{ "X-Claim-Token" = $tok }
   ```

**PASS gate**: 3, 4, 5 must all match expectations. Step 5 is the
most important — if STOP ever returns 423 something is wrong with the
endpoint exemption list.

---

## 6. Sign-off

When all sections PASS:

- [ ] Note today's date next to each completed section header.
- [ ] If any block was skipped or workaround'd, note why here.
- [ ] If `motion_graph.yaml` got new edges from §2 step 7, commit
  the file with a message referencing this verification run.
- [ ] If you want enforcement on permanently, set
  `XARM_ENFORCE_CLAIMS=true` in the NSSM service environment and
  `nssm restart xarm` so it survives reboots.

After this, the next code-side work in the pipeline:

- **Expand the motion graph** to cover the rest of the operational
  flows (UPLC plate handoff in particular). Use the `/control/graph/record`
  endpoint while running each sequence in advisory mode.
- **xArm skill catalog entry in `lab-skills`** so workflow code in
  other repos can see typed `Skill` objects (lives in `ac-organic-lab`,
  not here).

## Notes / scratch

(Use this section freely during the run — observed residual values,
unexpected gripper stroke matches, edges that turned out to need
different speeds, etc. I'll read it back if you ask me to follow up.)
