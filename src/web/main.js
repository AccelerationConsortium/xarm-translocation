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

    const homeBtn = document.getElementById('home-btn');
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
    const enableRobotBtn = document.getElementById('enable-robot-btn');
    const moveToStrokeBtn = document.getElementById('move-to-stroke-btn');
    const gripperStrokeInput = document.getElementById('gripper-stroke');
    const gripperStrokeRange = document.getElementById('gripper-stroke-range');
    const setGripperForceBtn = document.getElementById('set-gripper-force-btn');
    const gripperForceInput = document.getElementById('gripper-force');
    const gripperForceRange = document.getElementById('gripper-force-range');
    
    // Linear movement controls
    const linearStepsInput = document.getElementById('linear-steps');
    const moveLinearBtn = document.getElementById('move-linear-btn');

    let socket;
    let statusRefreshInterval = null;
    let isRobotMoving = false;
    let lastJointPositions = null;
    let movementDetectionThreshold = 0.1; // degrees

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
            if (body) {
                options.body = JSON.stringify(body);
            }
            const response = await fetch(`${API_BASE_URL}${endpoint}`, options);
            if (!response.ok) {
                let errorData = {};
                try {
                    errorData = await response.json();
                } catch {
                    errorData = {};
                }
                let errorMessage = errorData.detail || errorData.error || `HTTP error! status: ${response.status}`;
                
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
            track_position: trackMetric ? trackMetric.value : null,
            motion_graph: details.motion_graph || null,
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

            // Enable Robot button: armed when the device is reachable but
            // the arm itself is not currently "enabled" (typically `error`
            // after an emergency STOP, or `disabled`/`unknown`).
            updateEnableRobotBtn(componentStates.arm, isConnected);

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
            
            safeSetText('robot-mode', 'Hardware');

            safeSetText('last-error', systemStatus.last_error || 'None');

            // Live-feed joint angles into J1–Jn inputs (skip if user is focused on one)
            if (data.current_joints && Array.isArray(data.current_joints) && data.current_joints.length > 0) {
                const joints = data.current_joints;
                const numJoints = joints.length;

                // Show/hide J6 column based on robot model
                const j6col = document.getElementById('j6-col');
                if (j6col) j6col.style.display = numJoints >= 6 ? '' : 'none';

                jointInputIds.forEach((id, i) => {
                    if (i >= numJoints) return; // skip joints the robot doesn't have
                    const el = document.getElementById(id);
                    if (el && document.activeElement !== el) {
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
    const jointInputIds = ['j1-input','j2-input','j3-input','j4-input','j5-input','j6-input'];

    function updateEnableRobotBtn(armState, deviceReachable) {
        if (!enableRobotBtn) return;
        // Show the button as actionable when the device is reachable AND the
        // arm is in a state that motion_enable can recover from. After
        // emergency_stop the SDK callback flips arm to `error`; the spec
        // envelope reports `degraded` overall but the device is still up.
        const armOk = armState === 'enabled';
        const canEnable = deviceReachable && !armOk;
        enableRobotBtn.disabled = !canEnable;
        enableRobotBtn.textContent = 'Enable';
        if (canEnable) {
            enableRobotBtn.classList.remove('btn-secondary');
            enableRobotBtn.classList.add('btn-success');
        } else {
            enableRobotBtn.classList.remove('btn-success');
            enableRobotBtn.classList.add('btn-secondary');
        }
    }

    function setControlsState(enabled) {
        // Enable/disable control buttons based on connection state
        const controlButtons = [
            homeBtn, stopBtn, clearErrorsBtn, openGripperBtn, closeGripperBtn, enableGripperBtn, moveTrackLocBtn,
            movePredefinedBtn, moveToStrokeBtn, setGripperForceBtn, moveLinearBtn,
            moveJointsBtn,
            ...jogBtnIds.map(id => document.getElementById(id))
        ];

        // Enable/disable input fields
        const controlInputs = [
            jointSpeedInput, linearSpeedInput, trackSpeedInput, gripperStrokeInput, gripperForceInput, linearStepsInput,
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
        
        // Enable Robot button is now driven from the arm component state by
        // updateEnableRobotBtn() — see updateStatusUI. The wholesale
        // connect/disconnect toggle here would otherwise stomp on it
        // (after STOP the arm is `error` but the connection is still alive,
        // and we need the button to come BACK on, not stay off).
        
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
            console.log('WebSocket message received:', message); // Debug logging
            
            if (message.type === 'status_update') {
                // ``message.data`` is now a STATUS_SPEC v1.0 ``EquipmentStatus``
                // envelope; convert into the UI shape consumed by updateStatusUI.
                const uiData = envelopeToUiShape(message.data);
                if (document.readyState === 'complete') {
                    updateStatusUI(uiData);
                } else {
                    setTimeout(() => updateStatusUI(uiData), 100);
                }
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
            configSelect.innerHTML = profiles.map(p => `<option value="${p}">${p.replace(/_/g, ' ').toUpperCase()}</option>`).join('');
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
                const displayText = positions ? `${loc} (${positions}mm)` : loc;
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

    homeBtn.addEventListener('click', () => {
        apiRequest('/move/home', 'POST');
    });
    stopBtn.addEventListener('click', () => {
        apiRequest('/move/stop', 'POST');
    });
    clearErrorsBtn.addEventListener('click', () => {
        apiRequest('/clear/errors', 'POST');
    });
    
    enableRobotBtn.addEventListener('click', () => {
        apiRequest('/robot/enable', 'POST');
    });

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

    // Test log button
    const testLogBtn = document.getElementById('test-log-btn');
    if (testLogBtn) {
        testLogBtn.addEventListener('click', () => {
            console.log('Test log button clicked');
            apiRequest('/test/log', 'POST');
        });
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
        const numSteps = parseInt(linearStepsInput.value) || 1;
        const speed = linearSpeedInput ? parseInt(linearSpeedInput.value) || 100 : 100;
        
        if (!targetLocation) {
            showMessage('Please select a destination location.', 'error');
            return;
        }
        
        // Use new plate_linear endpoint - moves from current position to target
        // Tool maintains the same absolute orientation throughout movement
        apiRequest('/move/plate_linear', 'POST', {
            target_location: targetLocation,
            num_steps: numSteps,
            speed: speed,
            wait_between_steps: 0.1
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
            // Only collect joints that are visible (respects num_joints from robot)
            const j6col = document.getElementById('j6-col');
            const j6hidden = j6col && j6col.style.display === 'none';
            const activeIds = j6hidden ? jointInputIds.slice(0, 5) : jointInputIds;

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