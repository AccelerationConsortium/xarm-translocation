document.addEventListener('DOMContentLoaded', () => {
    // Create connection-details element if it doesn't exist
    if (!document.getElementById('connection-details')) {
        const statusContainer = document.getElementById('status-container');
        const connectionDetails = document.createElement('div');
        connectionDetails.id = 'connection-details';
        connectionDetails.className = 'connection-details';
        statusContainer.appendChild(connectionDetails);
    }
    
    const API_BASE_URL = `${window.location.protocol}//${window.location.host}`;
    const wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const wsHost = window.location.port === '8000'
        ? window.location.host
        : `${window.location.hostname}:8000`;
    const WS_URL = `${wsProtocol}://${wsHost}/ws`;

    const connectBtn = document.getElementById('connect-btn');
    const disconnectBtn = document.getElementById('disconnect-btn');
    const configSelect = document.getElementById('config-select');
    const safetyLevelSelect = document.getElementById('safety-level-select');
    const logStream = document.getElementById('log-stream');

    const stopBtn = document.getElementById('stop-btn');
    const clearErrorsBtn = document.getElementById('clear-errors-btn');

    const openGripperBtn = document.getElementById('open-gripper-btn');
    const closeGripperBtn = document.getElementById('close-gripper-btn');
    const enableGripperBtn = document.getElementById('enable-gripper-btn');

    const trackLocationSelect = document.getElementById('track-location-select');
    const moveTrackLocBtn = document.getElementById('move-track-loc-btn');
    const trackSpeedInput = document.getElementById('track-speed');
    const predefinedPositionSelect = document.getElementById('predefined-position-select');
    const movePredefinedBtn = document.getElementById('move-predefined-btn');
    const jointSpeedInput = document.getElementById('joint-speed');
    const linearSpeedInput = document.getElementById('linear-speed');
    const realtimeJointsDisplay = document.getElementById('realtime-joints');
    const manualModeSwitch = document.getElementById('manual-mode-switch');
    const manualModeCheckbox = document.getElementById('manual-mode-checkbox');
    const moveToStrokeBtn = document.getElementById('move-to-stroke-btn');
    const gripperStrokeInput = document.getElementById('gripper-stroke');
    const gripperStrokeRange = document.getElementById('gripper-stroke-range');
    const setGripperForceBtn = document.getElementById('set-gripper-force-btn');
    const gripperForceInput = document.getElementById('gripper-force');
    const gripperForceRange = document.getElementById('gripper-force-range');
    
    // Linear movement controls
    const moveLinearBtn = document.getElementById('move-linear-btn');

    // STATUS_SPEC v1.1 claim: the /web/ UI is a first-class claim holder.
    // The arm refuses motion (HTTP 423) unless the caller presents the
    // active claim's token, so an operator here must "Take Control" to
    // drive — which surfaces them as details.claimed_by and locks
    // workflows out (and vice-versa). STOP / Clear Errors stay ungated.
    const takeControlBtn = document.getElementById('take-control-btn');
    const claimStatusEl = document.getElementById('claim-status');
    const CLAIM_OWNER = 'human@xarm-web';
    const claimSessionId =
        (window.crypto && crypto.randomUUID && crypto.randomUUID()) ||
        `xarm-web-${Date.now()}-${Math.random().toString(16).slice(2)}`;
    let claimToken = null;          // non-null while this browser holds the claim
    let claimHeartbeatTimer = null; // setInterval handle for the heartbeat

    let socket;
    let statusRefreshInterval = null;
    let isRobotMoving = false;
    let lastJointPositions = null;
    // Joint-angle inputs double as the live readout AND the Move-Joints target.
    // Telemetry overwrites them every tick, but it must NOT clobber a value the
    // operator has typed-but-not-yet-submitted: focus alone is insufficient
    // because clicking "Move Joints" (or tabbing to the next joint) blurs the
    // box, and the next tick would revert the target to the current angle,
    // making the move a no-op. A box stays "dirty" from first edit until a move
    // is dispatched, at which point telemetry resumes (and animates toward the
    // target). Keyed by input id, e.g. "j1-input".
    const dirtyJoints = new Set();
    let movementDetectionThreshold = 0.1; // degrees
    // Timestamp of the last WebSocket status push. While pushes are fresh the
    // server's telemetry loop drives live updates, so we suppress the fast HTTP
    // polling escalation and keep only the cheap idle safety poll. If the
    // socket drops, this goes stale and HTTP polling auto-resumes.
    let lastPushAt = 0;
    const PUSH_STALE_MS = 1500;
    // Last values applied from the compact telemetry stream, so each ~10 Hz
    // tick only writes DOM that actually changed (vs. the full status render).
    const tlmLast = { status: undefined, manual: undefined, track: undefined };

    // --- API Helper ---
    async function apiRequest(endpoint, method = 'GET', body = null, skipErrorDisplay = false) {
        const requestLabel = `${method} ${endpoint}`;
        if (method !== 'GET') {
            addLogEntry(`API ${requestLabel}`, 'info');
        }

        try {
            const options = {
                method,
                headers: { 'Content-Type': 'application/json' },
            };
            // Attach the claim token so gated endpoints accept the request
            // while this browser holds control. Harmless when enforcement
            // is off or for ungated endpoints (the server ignores it).
            if (claimToken) {
                options.headers['X-Claim-Token'] = claimToken;
            }
            if (body) {
                options.body = JSON.stringify(body);
            }
            const response = await fetch(`${API_BASE_URL}${endpoint}`, options);
            if (!response.ok) {
                // 423 Locked: the claim is held elsewhere (or this browser's
                // claim expired/was stolen). Drop our local claim state so the
                // UI re-locks and the operator can re-take control.
                if (response.status === 423) {
                    handleClaimLost();
                }
                let errorData = {};
                try {
                    errorData = await response.json();
                } catch {
                    errorData = {};
                }
                let errorMessage = errorData.detail || errorData.error || `HTTP error! status: ${response.status}`;
                if (response.status === 423) {
                    const heldBy = errorData?.detail?.claimed_by?.owner;
                    errorMessage = heldBy
                        ? `Locked: control held by ${heldBy}. Click Take Control to request it.`
                        : 'Locked: you do not hold control. Click Take Control first.';
                }
                
                // Simplify connection error messages
                if (endpoint === '/connect' && response.status >= 500) {
                    errorMessage = 'Failed to initialize robot connection.';
                }
                
                throw new Error(errorMessage);
            }
            const data = await response.json();
            if (method !== 'GET') {
                addLogEntry(`OK ${requestLabel}`, 'info');
            }
            return data;
        } catch (error) {
            console.error(`API request failed: ${error.message}`);
            if (method !== 'GET') {
                addLogEntry(`FAILED ${requestLabel}: ${error.message}`, 'error');
            }
            
            // For connection errors, show error below status (unless skipped)
            if (!skipErrorDisplay) {
                showMessage(error.message, 'error');
            }
            return null;
        }
    }

    // --- Dynamic Refresh Rate Management ---
    function startMovementRefresh() {
        if (statusRefreshInterval) {
            clearInterval(statusRefreshInterval);
        }
        // 10Hz refresh during movement
        statusRefreshInterval = setInterval(fetchAndUpdateStatus, 100);
    }

    function startIdleRefresh() {
        if (statusRefreshInterval) {
            clearInterval(statusRefreshInterval);
        }
        // 0.5Hz refresh when idle
        statusRefreshInterval = setInterval(fetchAndUpdateStatus, 2000);
    }

    function stopRefresh() {
        if (statusRefreshInterval) {
            clearInterval(statusRefreshInterval);
            statusRefreshInterval = null;
        }
    }

    function detectMovement(currentJoints) {
        if (!lastJointPositions || !currentJoints || !Array.isArray(currentJoints)) {
            lastJointPositions = currentJoints;
            return false;
        }

        // Check if any joint has moved more than threshold
        const isMoving = currentJoints.some((joint, index) => {
            if (index >= lastJointPositions.length) return false;
            return Math.abs(joint - lastJointPositions[index]) > movementDetectionThreshold;
        });

        lastJointPositions = [...currentJoints];
        return isMoving;
    }

    function updateRefreshRate(currentJoints) {
        const wasMoving = isRobotMoving;
        isRobotMoving = detectMovement(currentJoints);

        // Grey-out and lock the joint inputs while the arm is actually moving,
        // restoring them when it settles. Done on the transition (not every
        // tick) and before the early-return below so it fires on both the WS
        // and HTTP-poll paths.
        if (isRobotMoving !== wasMoving) {
            applyMovingLock(isRobotMoving);
        }

        // While the WebSocket telemetry push is fresh, it already delivers
        // live motion at the server's rate — don't also escalate to 10 Hz HTTP
        // polling. Make sure we're on the cheap idle safety poll and bail.
        if (Date.now() - lastPushAt < PUSH_STALE_MS) {
            if (!statusRefreshInterval) startIdleRefresh();
            return;
        }

        // Only change refresh rate when movement state changes
        if (isRobotMoving && !wasMoving) {
            startMovementRefresh();
        } else if (!isRobotMoving && wasMoving) {
            // Add a small delay before switching to idle to avoid rapid switching
            setTimeout(() => {
                if (!isRobotMoving) { // Double-check we're still not moving
                    startIdleRefresh();
                }
            }, 500);
        }
    }

    // While the arm is moving, the J1–Jn boxes are a live readout only — grey
    // them and lock out editing so an operator can't type a target into a box
    // that telemetry is rewriting underneath them. When motion stops, restore
    // each box to whatever the other locks dictate by mirroring the Move-Joints
    // button, which is already gated on connection + claim + manual mode (and
    // is never touched by this lock). A typed-but-unsubmitted value (dirty box)
    // is preserved across the move: disabling never clears the value and
    // telemetry still skips dirty boxes.
    function applyMovingLock(moving) {
        const lockedByOthers = !!(moveJointsBtn && moveJointsBtn.disabled);
        jointInputIds.forEach(id => {
            const el = document.getElementById(id);
            if (!el) return;
            el.classList.toggle('moving-locked', moving);
            el.disabled = moving || lockedByOthers;
        });
    }

    // --- Compact telemetry (high-frequency live push) ---
    // Applies the small `telemetry` WS message with a field-diff so each ~10 Hz
    // tick only touches DOM that changed. The full `status_update` envelope
    // (connect + every action) still drives gripper/track buttons, the
    // motion-graph card, errors, etc.
    function applyTelemetry(d) {
        if (!d) return;

        // Live joint angles — the per-tick payload. Skip an input the user is
        // editing, and only write when the formatted value actually changes.
        const joints = d.current_joints;
        if (Array.isArray(joints) && joints.length > 0) {
            applyJointColumnVisibility(d.num_joints || joints.length);
            jointInputIds.forEach((id, i) => {
                if (i >= joints.length) return;
                const el = document.getElementById(id);
                if (el && document.activeElement !== el && !dirtyJoints.has(id)) {
                    const v = parseFloat(joints[i]).toFixed(2);
                    if (el.value !== v) el.value = v;
                }
            });
            if (realtimeJointsDisplay) {
                const txt = `[${joints.map(n => Number(n).toFixed(1)).join(', ')}]`;
                if (realtimeJointsDisplay.value !== txt) realtimeJointsDisplay.value = txt;
            }
            updateRefreshRate(joints);
        }

        // Track position — only on change.
        if (d.track_position !== tlmLast.track) {
            tlmLast.track = d.track_position;
            if (typeof d.track_position === 'number') {
                safeSetText('track-position-display', `${d.track_position.toFixed(2)} mm`);
            }
        }

        // Manual (drag/teach) mode — only on change. Drives the switch, the
        // XYZ/joint control lock-out, and the status-bar mode text.
        if (d.manual_mode !== tlmLast.manual) {
            tlmLast.manual = d.manual_mode;
            updateManualModeBtn(d.manual_mode === true, d.is_alive === true);
            applyManualModeLock(d.manual_mode === true);
            safeSetText('robot-mode', d.manual_mode === true ? 'Manual (drag)' : 'Position');
        }

        // Coarse state (status light/text + control enabling) — only on change,
        // so setControlsState isn't re-run every tick.
        if (d.equipment_status !== tlmLast.status) {
            tlmLast.status = d.equipment_status;
            const alive = d.is_alive === true;
            updateStatusText(alive
                ? (d.equipment_status ? `Connected (${d.equipment_status})` : 'Connected')
                : `Disconnected${d.equipment_status ? ` (${d.equipment_status})` : ''}`);
            setControlsState(alive);
            // setControlsState re-enables motion controls when alive; re-assert
            // the manual + claim locks so they stay disabled when manual mode is
            // on or this browser doesn't hold the claim.
            applyManualModeLock(tlmLast.manual === true);
            applyClaimLock(claimToken !== null);
            // ...and re-assert the moving lock, since setControlsState above
            // would otherwise re-enable the joint boxes mid-move.
            applyMovingLock(isRobotMoving);
        }
    }

    // --- Status Fetching ---
    // The backend now returns a STATUS_SPEC v1.0 ``EquipmentStatus`` envelope.
    // Pull the per-component state, metrics and details out of that shape and
    // pass a UI-shaped object into updateStatusUI.
    function envelopeToUiShape(envelope) {
        const components = envelope.components || {};
        const metrics = envelope.metrics || {};
        const details = envelope.details || {};
        const lastError = envelope.last_error;
        const trackMetric = metrics.track_position;

        // Treat ``ready``, ``busy``, ``degraded`` as "controller is up and the
        // UI's control buttons should be live". ``requires_init`` and ``error``
        // mean disabled controls.
        const status = envelope.equipment_status;
        const isAlive = ['ready', 'busy', 'degraded', 'e_stop'].includes(status);

        return {
            equipment_status: status,
            is_alive: isAlive,
            connection_details: details.connection_details || null,
            system_status: {
                last_error: lastError ? lastError.message : 'None',
            },
            component_states: {
                arm: components.arm ? components.arm.state : 'N/A',
                gripper: components.gripper ? components.gripper.state : 'N/A',
                track: components.track ? components.track.state : 'N/A',
            },
            current_position: details.current_position,
            current_joints: details.current_joints,
            num_joints: details.num_joints || null,
            track_position: trackMetric ? trackMetric.value : null,
            motion_graph: details.motion_graph || null,
            manual_mode: details.manual_mode === true,
            claimed_by: details.claimed_by || null,
        };
    }

    async function fetchAndUpdateStatus() {
        // Check if DOM is ready before proceeding
        if (document.readyState !== 'complete') {
            return;
        }

        try {
            const envelope = await apiRequest('/status', 'GET', null, true); // Skip error display
            if (envelope) {
                updateStatusUI(envelopeToUiShape(envelope));
            }
        } catch (error) {
            console.error('Failed to fetch status:', error);
            // On error, assume disconnected
            setControlsState(false);
            updateStatusText('Disconnected');
        }
    }

    // --- UI Updates ---
    function updateStatusUI(data) {
        try {
            // Quick check: if basic elements don't exist, DOM might not be ready
            if (!document.getElementById('arm-state') || !document.getElementById('status-text')) {
                console.error('DOM elements not ready, skipping status update');
                return;
            }
            
            // Check if all required DOM elements exist
            const requiredElements = [
                'arm-state', 'gripper-state', 'track-state', 'robot-mode', 'last-error'
            ];
            
            const missingElements = requiredElements.filter(id => {
                const element = document.getElementById(id);
                return !element;
            });
            
            if (missingElements.length > 0) {
                console.error('Missing required DOM elements:', missingElements);
                console.error('DOM ready state:', document.readyState);
                return;
            }
            
            // The `is_alive` flag is now derived from STATUS_SPEC equipment_status.
            const isConnected = data.is_alive === true;
            const equipmentStatus = data.equipment_status; // ready/busy/error/...

            // Update connection text and light
            try {
                if (isConnected) {
                    // Surface the spec state in the title (e.g. "Connected (busy)").
                    const label = equipmentStatus ? `Connected (${equipmentStatus})` : 'Connected';
                    updateStatusText(label);
                    if (data.connection_details) {
                        const details = data.connection_details;
                        const subtext = `${details.host}:${details.port} (${details.profile_name})`;
                        showMessage(subtext, 'info');
                    } else {
                        clearMessage();
                    }
                } else {
                    const label = equipmentStatus ? `Disconnected (${equipmentStatus})` : 'Disconnected';
                    updateStatusText(label);
                    clearMessage();
                }
            } catch (error) {
                console.error('Error updating connection status:', error);
            }
            
            // Update status grid with error handling
            const systemStatus = data.system_status || {};
            const componentStates = data.component_states || {};

            // Helper function to safely update element text
            const safeSetText = (elementId, text) => {
                try {
                    const element = document.getElementById(elementId);
                    
                    if (element) {
                        element.textContent = text;
                    } else {
                        console.error(`Element with id '${elementId}' not found`);
                    }
                } catch (error) {
                    console.error(`Unexpected error in safeSetText for '${elementId}':`, error);
                }
            };

            const setInputHelp = (element, text, disabled = false) => {
                if (!element) return;
                element.textContent = text;
                element.classList.toggle('is-disabled', disabled);
            };

            safeSetText('arm-state', componentStates.arm || 'N/A');
            
            // Update gripper state and name - show name regardless of state
            const gripperState = componentStates.gripper || 'N/A';
            const gripperConfig = data.connection_details?.gripper_config || {};
            const gripperName = gripperConfig.name || data.connection_details?.gripper_type || 'N/A';
            const hasStrokeControl = gripperConfig.has_stroke_control || false;
            const hasForceControl = gripperConfig.has_force_control || false;
            
            // The status-bar 'gripper-state' field is the canonical gripper
            // display. (This used to also write 'gripper-type-display' and
            // 'gripper-state-display', but those elements were removed from
            // index.html, so writing them only spammed "element not found".)
            if (gripperState === 'enabled' && data.connection_details?.gripper_type) {
                safeSetText('gripper-state', `${gripperName} (${gripperState})`);
                
                // Enable/disable stroke control based on gripper configuration
                if (hasStrokeControl) {
                    const strokeRange = gripperConfig.stroke_range || {};
                    const minStroke = strokeRange.min || 0;
                    const maxStroke = strokeRange.max || 800;
                    
                    if (gripperStrokeInput) {
                        gripperStrokeInput.disabled = false;
                        gripperStrokeInput.placeholder = `${minStroke}-${maxStroke}`;
                        gripperStrokeInput.min = minStroke.toString();
                        gripperStrokeInput.max = maxStroke.toString();
                    }
                    setInputHelp(gripperStrokeRange, `(${minStroke}–${maxStroke})`, false);
                    if (moveToStrokeBtn) {
                        moveToStrokeBtn.disabled = false;
                        moveToStrokeBtn.classList.remove('btn-secondary');
                        moveToStrokeBtn.classList.add('btn-primary');
                    }
                } else {
                    // No stroke control - disable and gray out
                    if (gripperStrokeInput) {
                        gripperStrokeInput.disabled = true;
                        gripperStrokeInput.placeholder = "";
                        gripperStrokeInput.value = "";
                    }
                    setInputHelp(gripperStrokeRange, '', true);
                    if (moveToStrokeBtn) {
                        moveToStrokeBtn.disabled = true;
                        moveToStrokeBtn.classList.remove('btn-primary');
                        moveToStrokeBtn.classList.add('btn-secondary');
                    }
                }

                if (hasForceControl) {
                    const forceRange = gripperConfig.force_range || {};
                    const minForce = forceRange.min || 1;
                    const maxForce = forceRange.max || 100;

                    if (gripperForceInput) {
                        gripperForceInput.disabled = false;
                        gripperForceInput.placeholder = `${minForce}-${maxForce}`;
                        gripperForceInput.min = minForce.toString();
                        gripperForceInput.max = maxForce.toString();
                        if (!gripperForceInput.value && gripperConfig.force) {
                            gripperForceInput.value = gripperConfig.force.toString();
                        }
                    }
                    setInputHelp(gripperForceRange, `(${minForce}–${maxForce})`, false);
                    if (setGripperForceBtn) {
                        setGripperForceBtn.disabled = false;
                        setGripperForceBtn.classList.remove('btn-secondary');
                        setGripperForceBtn.classList.add('btn-primary');
                    }
                } else {
                    if (gripperForceInput) {
                        gripperForceInput.disabled = true;
                        gripperForceInput.placeholder = "";
                        gripperForceInput.value = "";
                    }
                    setInputHelp(gripperForceRange, '', true);
                    if (setGripperForceBtn) {
                        setGripperForceBtn.disabled = true;
                        setGripperForceBtn.classList.remove('btn-primary');
                        setGripperForceBtn.classList.add('btn-secondary');
                    }
                }
            } else {
                // Keep the gripper name visible even when it isn't enabled.
                safeSetText('gripper-state', gripperName && gripperName !== 'N/A'
                    ? `${gripperName} (${gripperState})`
                    : gripperState);
                if (gripperStrokeInput) {
                    gripperStrokeInput.disabled = true;
                    gripperStrokeInput.placeholder = "";
                    gripperStrokeInput.value = "";
                }
                setInputHelp(gripperStrokeRange, '', true);
                if (moveToStrokeBtn) {
                    moveToStrokeBtn.disabled = true;
                    moveToStrokeBtn.classList.remove('btn-primary');
                    moveToStrokeBtn.classList.add('btn-secondary');
                }
                if (gripperForceInput) {
                    gripperForceInput.disabled = true;
                    gripperForceInput.placeholder = "";
                    gripperForceInput.value = "";
                }
                setInputHelp(gripperForceRange, '', true);
                if (setGripperForceBtn) {
                    setGripperForceBtn.disabled = true;
                    setGripperForceBtn.classList.remove('btn-primary');
                    setGripperForceBtn.classList.add('btn-secondary');
                }
            }

            // Manual (drag/teach) mode toggle reflects the live SDK mode.
            updateManualModeBtn(data.manual_mode === true, isConnected);

            // Update enable button state
            const enableGripperBtn = document.getElementById('enable-gripper-btn');
            if (enableGripperBtn) {
                if (gripperState === 'enabled') {
                    enableGripperBtn.textContent = 'Enable';
                    enableGripperBtn.disabled = true;
                    enableGripperBtn.classList.remove('btn-success');
                    enableGripperBtn.classList.add('btn-secondary');
                } else {
                    enableGripperBtn.textContent = 'Enable';
                    enableGripperBtn.disabled = false;
                    enableGripperBtn.classList.remove('btn-secondary');
                    enableGripperBtn.classList.add('btn-success');
                }
            }
            
            // Update track state to show position when enabled
            const trackState = componentStates.track || 'N/A';
            if (trackState === 'enabled' && data.track_position !== null && data.track_position !== undefined) {
                safeSetText('track-state', `${trackState} (${data.track_position.toFixed(2)}mm)`);
                safeSetText('track-position-display', `${data.track_position.toFixed(2)} mm`);
            } else {
                safeSetText('track-state', trackState);
                safeSetText('track-position-display', 'N/A');
            }
            
            safeSetText('robot-mode', data.manual_mode === true ? 'Manual (drag)' : 'Position');

            safeSetText('last-error', systemStatus.last_error || 'None');

            // Column visibility tracks the robot model (J1-J5 / J1-J6 / J1-J7).
            // Prefer the reported num_joints so columns are right even before
            // joint data arrives; fall back to the live joint count.
            const modelJoints = data.num_joints || (Array.isArray(data.current_joints) ? data.current_joints.length : 0);
            applyJointColumnVisibility(modelJoints);

            // Live-feed joint angles into J1–Jn inputs (skip if user is focused on one)
            if (data.current_joints && Array.isArray(data.current_joints) && data.current_joints.length > 0) {
                const joints = data.current_joints;
                const numJoints = joints.length;

                jointInputIds.forEach((id, i) => {
                    if (i >= numJoints) return; // skip joints the robot doesn't have
                    const el = document.getElementById(id);
                    if (el && document.activeElement !== el && !dirtyJoints.has(id)) {
                        el.value = parseFloat(joints[i]).toFixed(2);
                    }
                });

                // Update the compact realtime display
                if (realtimeJointsDisplay) {
                    realtimeJointsDisplay.value = `[${joints.map(n => Number(n).toFixed(1)).join(', ')}]`;
                }
                updateRefreshRate(joints);
            } else if (realtimeJointsDisplay) {
                realtimeJointsDisplay.value = '[No data]';
            }

            
            // Set the state of all controls based on the connection status
            try {
                setControlsState(isConnected);

                // Manual mode locks out XYZ + joint motion controls. Applied
                // after setControlsState so it wins over the connection-based
                // enabling (the joint-angle readout still updates live).
                applyManualModeLock(data.manual_mode === true);

                // Claim lock: gated controls stay disabled unless THIS browser
                // holds the claim. Applied last so it wins over the connection
                // enabling. STOP / Clear Errors are never claim-locked.
                applyClaimLock(claimToken !== null);
                updateClaimIndicator(data.claimed_by);
                // Re-assert the moving lock after the other locks have settled
                // moveJointsBtn's disabled state (which it mirrors).
                applyMovingLock(isRobotMoving);

                // Manage refresh rate based on connection state
                if (!isConnected && statusRefreshInterval) {
                    // Stop refreshing when disconnected to save resources
                    stopRefresh();
                } else if (isConnected && !statusRefreshInterval) {
                    // Resume refreshing when reconnected
                    startIdleRefresh();
                }
            } catch (error) {
                console.error('Error setting controls state:', error);
            }

            // Motion-graph card (Phase 4): show current node, reachable
            // destinations as buttons, current mode in the select.
            renderMotionGraphCard(data.motion_graph);

        } catch (error) {
            console.error('Fatal error in updateStatusUI:', error);
            console.error('Stack trace:', error.stack);
        }
    }

    // ── Motion Graph card (Phase 4) ─────────────────────────────────

    function renderMotionGraphCard(motionGraph) {
        const card = document.getElementById('motion-graph-card');
        if (!card) return;
        if (!motionGraph) {
            // No graph loaded — keep the card hidden.
            card.hidden = true;
            return;
        }
        card.hidden = false;

        const modeSelect = document.getElementById('mg-mode-select');
        const currentEl = document.getElementById('mg-current-node');
        const reachableEl = document.getElementById('mg-reachable');

        // Mode select reflects server state. Only update if it's not
        // currently focused (so the user can change it without us
        // stomping their selection mid-dropdown).
        if (modeSelect && document.activeElement !== modeSelect) {
            modeSelect.value = motionGraph.graph_mode || 'off';
        }

        // Current node + reachable buttons.
        const current = motionGraph.current_node;
        currentEl.textContent = current || '(off-grid)';

        reachableEl.innerHTML = '';
        const reachable = motionGraph.reachable_nodes || [];
        if (reachable.length === 0) {
            const span = document.createElement('span');
            span.className = 'muted';
            span.textContent = current ? '(no outgoing edges)' : '(off-grid — use Recover)';
            reachableEl.appendChild(span);
        } else {
            reachable.forEach(nodeId => {
                const btn = document.createElement('button');
                btn.className = 'btn btn-primary';
                btn.textContent = nodeId;
                btn.title = `Move to ${nodeId}`;
                btn.addEventListener('click', () => {
                    // POST node id to /control/graph/move_to so the
                    // server looks up the underlying arm-pose preset
                    // name. Node ids and preset names can differ
                    // (e.g. node 'uplc_draw_approach' arm 'uplc_draw_home').
                    apiRequest('/control/graph/move_to', 'POST', { node_id: nodeId });
                });
                reachableEl.appendChild(btn);
            });
        }
    }

    async function changeGraphMode(newMode) {
        const result = await apiRequest('/control/graph/mode', 'POST', { mode: newMode });
        if (result) {
            addLogEntry(`graph_mode -> ${result.graph_mode}`, 'info');
        }
        // Re-fetch status so the UI reflects the change.
        fetchAndUpdateStatus();
    }

    async function openRecoverPanel() {
        const panel = document.getElementById('mg-recover-panel');
        const suggestedEl = document.getElementById('mg-suggested-node');
        const residualsEl = document.getElementById('mg-residuals');
        const acceptBtn = document.getElementById('mg-recover-accept');
        const forceCheckbox = document.getElementById('mg-recover-force');

        const nearest = await apiRequest('/graph/nearest', 'GET', null, true);
        panel.hidden = false;
        if (!nearest || !nearest.suggested_node) {
            suggestedEl.textContent = '(no match)';
            residualsEl.textContent = '';
            acceptBtn.disabled = true;
            forceCheckbox.checked = false;
            return;
        }
        suggestedEl.textContent = nearest.suggested_node;
        const ar = nearest.arm_residual_deg !== null ? `arm Δ ${nearest.arm_residual_deg.toFixed(2)}°` : '';
        const rr = nearest.rail_residual_mm !== null ? `rail Δ ${nearest.rail_residual_mm.toFixed(2)}mm` : '';
        residualsEl.textContent = [ar, rr].filter(s => s).join(' · ');
        acceptBtn.disabled = false;
        acceptBtn.dataset.nodeId = nearest.suggested_node;
        acceptBtn.dataset.withinTolerance = String(nearest.within_tolerance);
        forceCheckbox.checked = !nearest.within_tolerance;
    }

    async function acceptRecover() {
        const acceptBtn = document.getElementById('mg-recover-accept');
        const forceCheckbox = document.getElementById('mg-recover-force');
        const nodeId = acceptBtn.dataset.nodeId;
        if (!nodeId) return;
        const result = await apiRequest('/control/graph/recover_to', 'POST', {
            node_id: nodeId,
            force: forceCheckbox.checked,
        });
        if (result) {
            addLogEntry(`recovered to ${result.recovered_to}`, 'info');
            document.getElementById('mg-recover-panel').hidden = true;
            fetchAndUpdateStatus();
        }
    }
    
    function updateStatusText(text) {
        const statusText = document.getElementById('status-text');
        const statusLight = document.getElementById('status-light');
        
        if (statusText) {
            try {
                statusText.textContent = text;
            } catch (error) {
                console.error('Error setting status text:', error);
            }
        } else {
            console.error("Element with id 'status-text' not found");
        }
        
        if (statusLight) {
            try {
                // Set status light based on connection state. The label may be
                // "Connected", "Connected (busy)", "Disconnected (requires_init)",
                // etc., so we just look at the first word.
                const lowerText = text.toLowerCase();
                if (lowerText.startsWith('connected')) {
                    statusLight.className = 'status-light online';
                } else {
                    statusLight.className = 'status-light offline';
                }
            } catch (error) {
                console.error('Error setting status light:', error);
            }
        } else {
            console.error("Element with id 'status-light' not found");
        }
        // Do not clear error message here; let it persist until next status change
    }

    // --- Message Helpers ---
    function showMessage(msg, type = 'info') {
        const messageDiv = document.getElementById('connection-details');
        if (messageDiv) {
            messageDiv.textContent = msg;
            messageDiv.className = type === 'error' ? 'error-message-error' : 'error-message-info';
        } else {
            console.error('connection-details element not found');
        }
    }
    function clearMessage() {
        const messageDiv = document.getElementById('connection-details');
        if (messageDiv) {
            messageDiv.textContent = '';
            messageDiv.className = '';
        } else {
            console.error('connection-details element not found');
        }
    }

    // --- Direct Motion Control refs ---
    const moveJointsBtn    = document.getElementById('move-joints-btn');
    const directJointSpeed = document.getElementById('direct-joint-speed');
    const jogStepInput     = document.getElementById('jog-step');
    const jogBtnIds = ['jog-x-plus','jog-x-minus','jog-y-plus','jog-y-minus','jog-z-plus','jog-z-minus'];
    const jointInputIds = ['j1-input','j2-input','j3-input','j4-input','j5-input','j6-input','j7-input'];

    // Mark a joint box "dirty" the moment the operator edits it, so telemetry
    // stops overwriting the typed target (see dirtyJoints declaration above).
    // The flag is cleared when a move is dispatched (or the box is reset to the
    // live value on Escape / blur-with-empty).
    jointInputIds.forEach(id => {
        const el = document.getElementById(id);
        if (!el) return;
        el.addEventListener('input', () => dirtyJoints.add(id));
        // Escape abandons the edit: drop the dirty flag so the next telemetry
        // tick restores the current angle.
        el.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') { dirtyJoints.delete(id); el.blur(); }
        });
    });

    // --- Copy Joints: build "[j1, j2, ...]" from the visible joint inputs ---
    const copyJointsBtn = document.getElementById('copy-joints-btn');

    // Reads only the columns the current model exposes (J6/J7 hidden on a 5/6-DOF
    // arm), and strips trailing zeros so the output matches joint_config.yaml
    // style (e.g. "180" not "180.00", but "202.51" kept). Returns null when a
    // value isn't ready yet (disconnected / no live data).
    function buildJointsString() {
        const vals = [];
        for (const id of jointInputIds) {
            const el = document.getElementById(id);
            if (!el) continue;
            const col = el.closest('.joint-col');
            if (col && getComputedStyle(col).display === 'none') continue; // joint the robot doesn't have
            const v = el.value.trim();
            if (v === '' || isNaN(parseFloat(v))) return null;
            vals.push(String(parseFloat(v)));
        }
        return vals.length ? '[' + vals.join(', ') + ']' : null;
    }

    // Clipboard with a fallback: navigator.clipboard needs a secure context
    // (https or localhost), but the panel is usually served over plain http
    // across the Tailnet, where it's unavailable — fall back to execCommand.
    async function copyTextToClipboard(text) {
        try {
            if (navigator.clipboard && window.isSecureContext) {
                await navigator.clipboard.writeText(text);
                return true;
            }
        } catch (e) { /* fall through */ }
        try {
            const ta = document.createElement('textarea');
            ta.value = text;
            ta.style.position = 'fixed';
            ta.style.opacity = '0';
            document.body.appendChild(ta);
            ta.focus(); ta.select();
            const ok = document.execCommand('copy');
            document.body.removeChild(ta);
            return ok;
        } catch (e) { return false; }
    }

    if (copyJointsBtn) {
        copyJointsBtn.addEventListener('click', async () => {
            const orig = copyJointsBtn.textContent;
            const s = buildJointsString();
            if (!s) {
                copyJointsBtn.textContent = 'No data';
                setTimeout(() => { copyJointsBtn.textContent = orig; }, 1200);
                return;
            }
            const ok = await copyTextToClipboard(s);
            copyJointsBtn.textContent = ok ? 'Copied!' : 'Copy failed';
            setTimeout(() => { copyJointsBtn.textContent = orig; }, 1200);
        });
    }

    // Show exactly J1..numJoints columns (J1-J5 always present; J6/J7 are
    // model-dependent: xArm5 -> 5, xArm6 -> 6, xArm7 -> 7). Driven by the
    // robot model so the right columns show as soon as we're connected,
    // before live joint data arrives.
    function applyJointColumnVisibility(numJoints) {
        if (!numJoints) return;
        [['j6-col', 6], ['j7-col', 7]].forEach(([colId, jointN]) => {
            const col = document.getElementById(colId);
            if (col) col.style.display = numJoints >= jointN ? '' : 'none';
        });
    }

    function updateManualModeBtn(isManual, deviceReachable) {
        if (!manualModeCheckbox) return;
        // The toggle is live whenever the device is reachable. When manual
        // mode is on the switch slides + turns red (CSS .is-on) to signal the
        // joint brakes are released. The checkbox carries the real SDK state,
        // reconciled from /status on every poll so an optimistic flip that the
        // device rejects snaps back.
        manualModeCheckbox.disabled = !deviceReachable;
        manualModeCheckbox.checked = isManual;
        if (manualModeSwitch) {
            manualModeSwitch.classList.toggle('is-on', isManual);
            manualModeSwitch.classList.toggle('is-disabled', !deviceReachable);
        }
    }

    // While manual (drag/teach) mode is engaged the arm is back-drivable and
    // any commanded XYZ / joint motion is both meaningless and unsafe, so we
    // lock those controls out. Gripper, track, Home/Stop/Clear/Enable stay
    // live. The joint-angle inputs are disabled for *editing* only — the live
    // status feed keeps writing their .value, so the readout stays realtime.
    function applyManualModeLock(isManual) {
        const lockEls = [
            movePredefinedBtn, moveLinearBtn, moveJointsBtn,
            jointSpeedInput, linearSpeedInput,
            directJointSpeed, jogStepInput, predefinedPositionSelect,
            ...jogBtnIds.map(id => document.getElementById(id)),
            ...jointInputIds.map(id => document.getElementById(id)),
        ];
        lockEls.forEach(el => {
            if (!el) return;
            if (isManual) {
                el.disabled = true;
                el.classList.add('manual-locked');
            } else {
                // Leave .disabled as setControlsState() set it for the current
                // connection state; just drop the locked styling.
                el.classList.remove('manual-locked');
            }
        });
    }

    // --- STATUS_SPEC v1.1 claim lifecycle ---------------------------------
    // Claim-gated controls: everything that can command motion. STOP and
    // Clear Errors are deliberately excluded (they're ungated server-side
    // too — the safety floor must work even when control is held elsewhere).
    // Connect / Disconnect are also excluded (you connect before claiming).
    function claimGatedElements() {
        // NB: takeControlBtn, stopBtn, clearErrorsBtn, connect/disconnect are
        // intentionally NOT in this list.
        return [
            openGripperBtn, closeGripperBtn, enableGripperBtn,
            moveTrackLocBtn, movePredefinedBtn, moveToStrokeBtn,
            setGripperForceBtn, moveLinearBtn, moveJointsBtn,
            jointSpeedInput, linearSpeedInput, trackSpeedInput,
            gripperStrokeInput, gripperForceInput,
            directJointSpeed, jogStepInput,
            predefinedPositionSelect, trackLocationSelect,
            manualModeCheckbox,
            ...jogBtnIds.map(id => document.getElementById(id)),
            ...jointInputIds.map(id => document.getElementById(id)),
        ];
    }

    // Disable the motion-control surface unless this browser holds the claim.
    // When held, only the locked styling is dropped; .disabled is left to the
    // connection/status logic (so disconnected stays disabled). Mirrors
    // applyManualModeLock and must run AFTER setControlsState.
    function applyClaimLock(held) {
        claimGatedElements().forEach(el => {
            if (!el) return;
            if (!held) {
                el.disabled = true;
                el.classList.add('claim-locked');
            } else {
                el.classList.remove('claim-locked');
            }
        });
        // When locked, grey the manual switch too. When held, leave the
        // is-disabled class to updateManualModeBtn (which owns the
        // device-reachability case).
        if (!held && manualModeSwitch) {
            manualModeSwitch.classList.add('is-disabled');
        }
    }

    // Reflect the device-reported claim holder (details.claimed_by) in the
    // status header. "(you)" when the holder matches our session.
    function updateClaimIndicator(claimedBy) {
        if (!claimStatusEl) return;
        claimStatusEl.classList.remove('claim-free', 'claim-mine', 'claim-other');
        if (!claimedBy) {
            claimStatusEl.textContent = 'Control: nobody (open)';
            claimStatusEl.classList.add('claim-free');
            return;
        }
        const mine = claimedBy.session_id === claimSessionId;
        claimStatusEl.textContent = `Control: ${claimedBy.owner}${mine ? ' (you)' : ''}`;
        claimStatusEl.classList.add(mine ? 'claim-mine' : 'claim-other');
    }

    function updateTakeControlBtn() {
        if (!takeControlBtn) return;
        const held = claimToken !== null;
        takeControlBtn.textContent = held ? 'Release Control' : 'Take Control';
        takeControlBtn.classList.toggle('is-holding', held);
    }

    function startClaimHeartbeat(intervalSeconds) {
        stopClaimHeartbeat();
        // Beat at half the server's advertised interval so a single missed
        // beat (network hiccup) doesn't drop the lock. Floor at 2 s.
        const everyMs = Math.max(2000, ((intervalSeconds || 10) * 1000) / 2);
        claimHeartbeatTimer = setInterval(async () => {
            if (!claimToken) return;
            try {
                const r = await fetch(`${API_BASE_URL}/control/heartbeat`, {
                    method: 'POST',
                    headers: { 'X-Claim-Token': claimToken },
                });
                // 401/404 => the device forgot our claim (expiry or restart).
                if (r.status === 401 || r.status === 404) handleClaimLost();
            } catch (e) {
                // Transient network error: let the next beat retry. A real loss
                // surfaces as a 423 on the next action or a 401/404 next beat.
                console.warn('Heartbeat failed (will retry):', e);
            }
        }, everyMs);
    }

    function stopClaimHeartbeat() {
        if (claimHeartbeatTimer) {
            clearInterval(claimHeartbeatTimer);
            claimHeartbeatTimer = null;
        }
    }

    async function takeControl() {
        try {
            const resp = await fetch(`${API_BASE_URL}/control/claim`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    owner: CLAIM_OWNER,
                    session_id: claimSessionId,
                    ttl_s: 30,
                }),
            });
            if (resp.status === 200) {
                const data = await resp.json();
                claimToken = data.claim_token;
                startClaimHeartbeat(data.heartbeat_interval_s);
                updateTakeControlBtn();
                applyClaimLock(true);
                addLogEntry('Control acquired (human@xarm-web)', 'info');
                clearMessage();
                // Re-run the enable logic so gated controls light up now.
                fetchAndUpdateStatus();
            } else if (resp.status === 409) {
                let d = {};
                try { d = await resp.json(); } catch { /* noop */ }
                const owner = d.claimed_by && d.claimed_by.owner;
                showMessage(
                    `Cannot take control: held by ${owner || 'another session'}. Try again later.`,
                    'error',
                );
                addLogEntry(`Take Control refused (held by ${owner || 'another session'})`, 'warning');
            } else {
                showMessage(`Failed to take control (HTTP ${resp.status}).`, 'error');
            }
        } catch (e) {
            showMessage(`Failed to take control: ${e.message}`, 'error');
        }
    }

    // Release the claim. `viaUnload` uses fetch keepalive so the request
    // survives the page closing (sendBeacon can't set the X-Claim-Token header).
    async function releaseControl(viaUnload = false) {
        const token = claimToken;
        stopClaimHeartbeat();
        claimToken = null;
        updateTakeControlBtn();
        if (!viaUnload) {
            applyClaimLock(false);
            updateClaimIndicator(null);
        }
        if (!token) return;
        try {
            await fetch(`${API_BASE_URL}/control/release`, {
                method: 'POST',
                headers: { 'X-Claim-Token': token },
                keepalive: viaUnload,
            });
        } catch (e) {
            if (!viaUnload) console.warn('Release failed:', e);
        }
        if (!viaUnload) {
            addLogEntry('Control released', 'info');
            fetchAndUpdateStatus();
        }
    }

    // Called when the claim is gone from under us (423 on an action, or
    // 401/404 on a heartbeat). Drop local state and re-lock the UI.
    function handleClaimLost() {
        if (claimToken === null) return; // already released/lost
        claimToken = null;
        stopClaimHeartbeat();
        updateTakeControlBtn();
        applyClaimLock(false);
        showMessage(
            'Lost control — the claim is no longer held by this browser. Click Take Control to resume.',
            'error',
        );
        addLogEntry('Claim lost (expired or held elsewhere)', 'error');
    }

    function setControlsState(enabled) {
        // Enable/disable control buttons based on connection state.
        // takeControlBtn is here (not claim-gated) because acquiring a claim
        // needs a connected controller, but it must NOT require already
        // holding the claim.
        const controlButtons = [
            takeControlBtn,
            stopBtn, clearErrorsBtn, openGripperBtn, closeGripperBtn, enableGripperBtn, moveTrackLocBtn,
            movePredefinedBtn, moveToStrokeBtn, setGripperForceBtn, moveLinearBtn,
            moveJointsBtn,
            ...jogBtnIds.map(id => document.getElementById(id))
        ];

        // Enable/disable input fields
        const controlInputs = [
            jointSpeedInput, linearSpeedInput, trackSpeedInput, gripperStrokeInput, gripperForceInput,
            directJointSpeed, jogStepInput,
            ...jointInputIds.map(id => document.getElementById(id))
        ];
        
        // Enable/disable select dropdowns  
        const controlSelects = [
            predefinedPositionSelect, trackLocationSelect
        ];
        
        controlButtons.forEach(btn => {
            if (btn) {
                btn.disabled = !enabled;
            }
        });
        
        controlInputs.forEach(input => {
            if (input) {
                input.disabled = !enabled;
            }
        });
        
        controlSelects.forEach(select => {
            if (select) {
                select.disabled = !enabled;
            }
        });
        
        // Connect button: enabled when disconnected, disabled when connected
        if (connectBtn) {
            connectBtn.disabled = enabled;
        } else {
            console.error("Connect button element not found");
        }
        
        // Disconnect button: enabled when connected, disabled when disconnected
        if (disconnectBtn) {
            disconnectBtn.disabled = !enabled;
        } else {
            console.error("Disconnect button element not found");
        }
    }

    // --- WebSocket Handling ---
    function connectWebSocket() {
        if (socket && socket.readyState === WebSocket.OPEN) {
            return;
        }
        socket = new WebSocket(WS_URL);
        socket.onopen = () => console.log('WebSocket connected.');
        socket.onmessage = (event) => {
            const message = JSON.parse(event.data);

            if (message.type === 'status_update') {
                // ``message.data`` is now a STATUS_SPEC v1.0 ``EquipmentStatus``
                // envelope; convert into the UI shape consumed by updateStatusUI.
                // Mark the push as fresh so updateRefreshRate keeps HTTP polling
                // on the idle safety cadence (the push drives live motion).
                lastPushAt = Date.now();
                const uiData = envelopeToUiShape(message.data);
                // A full envelope is authoritative — resync the telemetry diff
                // cache so the next compact tick doesn't skip a changed field.
                tlmLast.status = uiData.equipment_status;
                tlmLast.manual = uiData.manual_mode;
                tlmLast.track = uiData.track_position;
                if (document.readyState === 'complete') {
                    updateStatusUI(uiData);
                } else {
                    setTimeout(() => updateStatusUI(uiData), 100);
                }
            } else if (message.type === 'telemetry') {
                // Compact high-frequency live push: cheap field-diff update.
                lastPushAt = Date.now();
                applyTelemetry(message.data);
            } else if (message.type === 'log') {
                // Handle incoming log messages from API server
                console.log('Log message received:', message.log_message); // Debug logging
                addLogEntry(message.log_message, message.log_type);
            }
        };
        socket.onclose = () => {
            console.log('WebSocket disconnected.');
            // Fetch current status from API instead of assuming disconnected
            setTimeout(() => fetchAndUpdateStatus(), 100);
        };
        socket.onerror = (error) => console.error('WebSocket error:', error);
    }

    // --- Initial Data Loading ---
    async function loadInitialData() {
        // Load connection profiles
        const profiles = await apiRequest('/api/configurations');
        if (profiles) {
            // Friendly display names for known profiles; fall back to a humanized
            // version of the raw profile name. Option values stay the raw profile
            // name so /connect keeps receiving robot / docker.
            const profileLabels = { robot: 'Robot', docker: 'Docker' };
            const label = p => profileLabels[p] || p.replace(/_/g, ' ').toUpperCase();
            configSelect.innerHTML = profiles.map(p =>
                `<option value="${p}"${p === 'robot' ? ' selected' : ''}>${label(p)}</option>`
            ).join('');
        }

        // Load arm locations with position values for both dropdowns
        const armLocations = await apiRequest('/locations');
        if (armLocations && armLocations.locations) {
            // Populate predefined positions dropdown
            predefinedPositionSelect.innerHTML = armLocations.locations.map(loc => {
                // Get position values if available
                const positions = armLocations.positions ? armLocations.positions[loc] : null;
                const displayText = positions ? `${loc} [${positions.join(', ')}]` : loc;
                return `<option value="${loc}">${displayText}</option>`;
            }).join('');
            
            // Linear movement now uses the same dropdown as Move Joints
        }
        
        // Load track locations with position values
        const trackLocations = await apiRequest('/track/locations');
        if (trackLocations && trackLocations.locations) {
            trackLocationSelect.innerHTML = trackLocations.locations.map(loc => {
                // Get position values if available
                const positions = trackLocations.positions ? trackLocations.positions[loc] : null;
                const displayText = positions ? `${loc} (${positions} mm)` : loc;
                return `<option value="${loc}">${displayText}</option>`;
            }).join('');
        }
    }
    
    // --- Event Listeners ---
    connectBtn.addEventListener('click', async () => {
        const selectedProfile = configSelect.value;

        // Disable connect button during connection attempt
        connectBtn.disabled = true;

        const body = {
            profile_name: selectedProfile,
            safety_level: safetyLevelSelect.value,
        };

        const response = await apiRequest('/connect', 'POST', body);
        
        if (response && response.message) {
            // Connection successful - fetch updated status from API
            setTimeout(() => fetchAndUpdateStatus(), 500);
            
            // Start WebSocket for real-time updates
            connectWebSocket();
        } else {
            // Connection failed - re-enable connect button and fetch current status
            setTimeout(() => fetchAndUpdateStatus(), 100);
        }
    });

    disconnectBtn.addEventListener('click', async () => {
        const response = await apiRequest('/disconnect', 'POST');
        
        // Close WebSocket connection first to prevent it from overriding our status update
        if (socket) socket.close();
        
        // Fetch updated status from API to update UI
        setTimeout(() => fetchAndUpdateStatus(), 100);
        
        if (response && response.message) {
            console.log('Disconnect response:', response.message);
        }
    });

    if (takeControlBtn) {
        takeControlBtn.addEventListener('click', () => {
            // The button toggles: take when free, release when held.
            if (claimToken !== null) {
                releaseControl();
            } else {
                takeControl();
            }
        });
    }

    // Release the claim when the operator leaves so it doesn't linger until
    // TTL expiry and block workflows. `pagehide` is the reliable mobile/desktop
    // unload signal; keepalive lets the request (with its header) survive.
    window.addEventListener('pagehide', () => {
        if (claimToken !== null) releaseControl(true);
    });

    stopBtn.addEventListener('click', () => {
        apiRequest('/move/stop', 'POST');
    });
    clearErrorsBtn.addEventListener('click', () => {
        apiRequest('/clear/errors', 'POST');
    });

    if (manualModeCheckbox) {
        manualModeCheckbox.addEventListener('change', () => {
            const enable = manualModeCheckbox.checked;
            if (enable && !confirm(
                'Enable Manual Mode?\n\n' +
                'This releases the joint brakes so the arm can be moved by hand. ' +
                'Support the arm before continuing — it may sag under its own ' +
                'weight or payload.\n\n' +
                'XYZ and joint controls are locked while manual mode is on.'
            )) {
                // User backed out — revert the optimistic flip immediately;
                // the next /status poll would also correct it.
                manualModeCheckbox.checked = false;
                if (manualModeSwitch) manualModeSwitch.classList.remove('is-on');
                return;
            }
            apiRequest('/robot/manual', 'POST', { enable });
        });
    }

    // Motion-graph card listeners (Phase 4). All elements may be
    // missing if index.html is older than this build — guard each one.
    const mgModeSelect = document.getElementById('mg-mode-select');
    if (mgModeSelect) {
        mgModeSelect.addEventListener('change', (e) => changeGraphMode(e.target.value));
    }
    const mgRecoverBtn = document.getElementById('mg-recover-btn');
    if (mgRecoverBtn) {
        mgRecoverBtn.addEventListener('click', openRecoverPanel);
    }
    const mgRecoverAccept = document.getElementById('mg-recover-accept');
    if (mgRecoverAccept) {
        mgRecoverAccept.addEventListener('click', acceptRecover);
    }

    function currentGripperForce() {
        if (!gripperForceInput || gripperForceInput.disabled || !gripperForceInput.value) {
            return null;
        }
        const force = parseFloat(gripperForceInput.value);
        return Number.isNaN(force) ? null : force;
    }

    openGripperBtn.addEventListener('click', () => apiRequest('/gripper/open', 'POST', { force: currentGripperForce() }));
    closeGripperBtn.addEventListener('click', () => apiRequest('/gripper/close', 'POST', { force: currentGripperForce() }));
    
    enableGripperBtn.addEventListener('click', () => {
        apiRequest('/component/enable', 'POST', { component: 'gripper' });
    });
    
    moveTrackLocBtn.addEventListener('click', () => {
        const location_name = trackLocationSelect.value;
        const speed = parseFloat(trackSpeedInput.value) || null;
        apiRequest('/track/move/location', 'POST', { location_name, speed });
    });

    movePredefinedBtn.addEventListener('click', () => {
        const location_name = predefinedPositionSelect.value;
        const speed = jointSpeedInput ? parseInt(jointSpeedInput.value) || 20 : 20;
        apiRequest('/move/location', 'POST', { 
            location_name, 
            speed 
        });
    });

    // Linear movement event listener
    moveLinearBtn.addEventListener('click', () => {
        const targetLocation = predefinedPositionSelect.value; // Use same dropdown as Move Joints
        const speed = linearSpeedInput ? parseInt(linearSpeedInput.value) || 100 : 100;

        if (!targetLocation) {
            showMessage('Please select a destination location.', 'error');
            return;
        }

        // Use new plate_linear endpoint - moves from current position to target
        // Tool maintains the same absolute orientation throughout movement
        apiRequest('/move/plate_linear', 'POST', {
            target_location: targetLocation,
            speed: speed
        });
    });

    moveToStrokeBtn.addEventListener('click', () => {
        const stroke = parseFloat(gripperStrokeInput.value);
        const min = parseFloat(gripperStrokeInput.min) || 0;
        const max = parseFloat(gripperStrokeInput.max) || 1000;
        
        if (isNaN(stroke)) {
            showMessage('Please enter a valid stroke value.', 'error');
            return;
        }
        
        if (stroke < min || stroke > max) {
            showMessage(`Stroke value must be between ${min} and ${max}.`, 'error');
            return;
        }
        
        apiRequest('/gripper/move/stroke', 'POST', { stroke, force: currentGripperForce() });
    });

    setGripperForceBtn.addEventListener('click', () => {
        const force = parseFloat(gripperForceInput.value);
        const min = parseFloat(gripperForceInput.min) || 1;
        const max = parseFloat(gripperForceInput.max) || 100;

        if (isNaN(force)) {
            showMessage('Please enter a valid gripper force.', 'error');
            return;
        }

        if (force < min || force > max) {
            showMessage(`Force value must be between ${min} and ${max}.`, 'error');
            return;
        }

        apiRequest('/gripper/force', 'POST', { force });
    });
    
    // --- Direct Motion Control handlers ---

    if (moveJointsBtn) {
        moveJointsBtn.addEventListener('click', () => {
            // Only collect joints whose column is visible (respects the robot
            // model: J1-J5 always, J6/J7 only when their column is shown).
            const activeIds = jointInputIds.filter(id => {
                const col = document.getElementById(id)?.closest('.joint-col');
                return !col || col.style.display !== 'none';
            });

            const angles = activeIds.map(id => {
                const v = parseFloat(document.getElementById(id)?.value);
                return Number.isNaN(v) ? null : v;
            });
            if (angles.some(v => v === null)) {
                showMessage('Enter all joint angles before moving.', 'error');
                return;
            }
            const speed = parseFloat(directJointSpeed?.value) || 10;
            apiRequest('/move/joints', 'POST', { angles, speed });
            // Target dispatched — let telemetry take the inputs back so they
            // animate toward the commanded angles.
            dirtyJoints.clear();
        });
    }

    // --- XYZ Jog handlers ---
    function jog(dx, dy, dz) {
        const step = parseFloat(jogStepInput?.value) || 10;
        const speed = parseFloat(linearSpeedInput?.value) || 100;
        apiRequest('/move/relative', 'POST', { dx: dx * step, dy: dy * step, dz: dz * step, speed });
    }

    const jogMap = {
        'jog-x-plus':  [  1,  0,  0 ],
        'jog-x-minus': [ -1,  0,  0 ],
        'jog-y-plus':  [  0,  1,  0 ],
        'jog-y-minus': [  0, -1,  0 ],
        'jog-z-plus':  [  0,  0,  1 ],
        'jog-z-minus': [  0,  0, -1 ],
    };

    Object.entries(jogMap).forEach(([id, [dx, dy, dz]]) => {
        const btn = document.getElementById(id);
        if (btn) btn.addEventListener('click', () => jog(dx, dy, dz));
    });

    // --- Initialization ---
    // Set initial disconnected state explicitly
    updateStatusText('Disconnected');
    setControlsState(false);
    
    // Then load initial data and establish connections
    connectWebSocket();
    loadInitialData().catch(console.error);
    
    // Finally check status (this will update if a robot is actually connected)
    fetchAndUpdateStatus();
    
    // Initialize real-time joints display
    if (realtimeJointsDisplay) {
        realtimeJointsDisplay.value = '[No data]';
    }
    
    // Don't start automatic refresh until connected - it will be started in updateStatusUI when needed
    
    // Log streaming functions
    function addLogEntry(message, type = 'info') {
        if (!logStream) return;
        
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;
        const timeSpan = document.createElement('span');
        timeSpan.className = 'log-time';
        timeSpan.textContent = `[${timestamp}]`;
        logEntry.appendChild(timeSpan);
        logEntry.appendChild(document.createTextNode(` ${message}`));
        
        // Add to top (newest first)
        logStream.insertBefore(logEntry, logStream.firstChild);
        
        // Keep only last 50 entries (fewer since box is smaller)
        const entries = logStream.querySelectorAll('.log-entry');
        if (entries.length > 50) {
            entries[entries.length - 1].remove();
        }
    }

    // Initialize logging
    addLogEntry('System initialized', 'info');
    
    window.apiRequest = apiRequest;
}); 