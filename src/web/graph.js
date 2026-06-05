/* Motion Graph viewer (Phase A: read-only render + live state).
 *
 * Fetches GET /graph (proxied through the web server to the API on :8000),
 * renders the node/edge topology with Cytoscape, and tracks where the arm
 * is live by subscribing to the same /ws status stream the control panel
 * uses. No claim and no writes in Phase A. */
(function () {
    'use strict';

    // HTTP goes through the page's own origin (the web server proxies /graph
    // and /control to the API). The WebSocket can't be proxied by the simple
    // http.server, so it connects straight to the API on :8000 — mirroring
    // main.js's logic so the page works whether served on :6001 or :8000.
    var API_BASE = window.location.protocol + '//' + window.location.host;
    var wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    var wsHost = window.location.port === '8000'
        ? window.location.host
        : window.location.hostname + ':8000';
    var WS_URL = wsProtocol + '://' + wsHost + '/ws';

    // Live-push freshness: while WS pushes arrive the fallback HTTP poll for
    // live state stays idle (it only fires when the socket goes quiet).
    var PUSH_STALE_MS = 2000;
    var lastPushAt = 0;

    var cy = null;
    var gripperStrokes = {};    // gripper_state name -> stroke (for grip defaults)
    var expanded = {};          // station -> true when its nodes are shown
    var autoStation = null;     // station auto-expanded to follow the robot
    var latestClaimedBy = null; // device claim holder from /status (for the lock)
    var currentNodeId = null;   // last applied live current_node
    var traverseTimer = null;

    var modeEl = document.getElementById('graph-mode');
    var currentEl = document.getElementById('graph-current');
    var payloadEl = document.getElementById('graph-payload');
    var claimEl = document.getElementById('claim-status');
    var messageEl = document.getElementById('graph-message');

    // Distinct, readable colours assigned to stations (a node's first tag) in
    // first-seen order. Falls back to a neutral grey once the palette is spent.
    var STATION_PALETTE = [
        '#0369a1', '#7c3aed', '#0d9488', '#b45309',
        '#be123c', '#4f46e5', '#15803d', '#a16207',
    ];
    var stationColors = {};
    var paletteCursor = 0;

    function colorForStation(station) {
        if (!station) return '#64748b';
        if (!(station in stationColors)) {
            stationColors[station] =
                STATION_PALETTE[paletteCursor % STATION_PALETTE.length] || '#64748b';
            paletteCursor += 1;
        }
        return stationColors[station];
    }

    function showMessage(text) {
        if (!messageEl) return;
        messageEl.textContent = text;
        messageEl.hidden = false;
    }
    function clearMessage() {
        if (messageEl) messageEl.hidden = true;
    }

    // ── Element construction ─────────────────────────────────────────

    // Cytoscape node data for one graph node. ▣ marks nodes whose state
    // holds labware, so the pick/place ladder is legible even before a move.
    function nodeData(n) {
        var station = (n.tags && n.tags.length) ? n.tags[0] : '';
        var held = n.payload && n.payload !== 'empty';
        var d = {
            id: n.id,
            label: held ? n.id + ' ▣' : n.id,
            station: station,
            color: colorForStation(station),
            rail: n.rail,
            payload: n.payload,
        };
        return d;
    }

    function buildElements(data) {
        var elements = [];
        var nodes = data.nodes || [];
        // One clickable "station tile" per station, labelled with its count.
        // Collapsing a station hides its member nodes and leaves just the tile.
        var counts = {};
        nodes.forEach(function (n) {
            var s = (n.tags && n.tags.length) ? n.tags[0] : '';
            if (s) counts[s] = (counts[s] || 0) + 1;
        });
        Object.keys(counts).forEach(function (s) {
            elements.push({ group: 'nodes', data: {
                id: 'grp:' + s, label: '▸ ' + s + ' (' + counts[s] + ')',
                isGroup: true, station: s, color: colorForStation(s),
                count: counts[s],
            } });
        });
        nodes.forEach(function (n) {
            elements.push({ group: 'nodes', data: nodeData(n) });
        });
        (data.edges || []).forEach(function (e) {
            var kind = e.grips ? 'grip' : (e.releases ? 'release' : 'plain');
            var precos = e.preconditions || [];
            elements.push({
                group: 'edges',
                data: {
                    id: 'e:' + e.from + '->' + e.to,
                    source: e.from,
                    target: e.to,
                    label: edgeLabel(e.mode, e.speed, precos),
                    kind: kind,
                    mode: e.mode,
                    speed: e.speed,
                    preconditions: precos,
                },
            });
        });
        return elements;
    }

    // Edge label: "mode @ speed" plus any preconditions appended after a
    // dot, e.g. "linear @ 15 · gripper_empty". Preconditions show on the
    // edge so they're visible at a glance (they're not editable here).
    function edgeLabel(mode, speed, precos) {
        var label = mode + (speed != null ? ' @ ' + speed : '');
        if (precos && precos.length) label += ' · ' + precos.join(', ');
        return label;
    }

    var CY_STYLE = [
        {
            selector: 'node',
            style: {
                'background-color': 'data(color)',
                'label': 'data(label)',
                'color': '#ffffff',
                'font-size': 11,
                'font-weight': 600,
                'text-valign': 'center',
                'text-halign': 'center',
                'text-wrap': 'wrap',
                'text-max-width': 120,
                'width': 'label',
                'height': 'label',
                'padding': '10px',
                'shape': 'round-rectangle',
                'border-width': 2,
                'border-color': 'rgba(15,23,42,0.15)',
            },
        },
        {
            // Station tile: a larger solid station-coloured pill you click to
            // show/hide that station's nodes.
            selector: 'node[?isGroup]',
            style: {
                'background-color': 'data(color)',
                'color': '#ffffff',
                'font-size': 14,
                'font-weight': 800,
                'shape': 'round-rectangle',
                'padding': '16px',
                'border-width': 3,
                'border-color': 'rgba(15,23,42,0.25)',
            },
        },
        {
            selector: 'node.current',
            style: {
                'background-color': '#047857',
                'border-width': 4,
                'border-color': '#065f46',
                'color': '#ffffff',
            },
        },
        {
            selector: 'node.holding',
            style: {
                'border-width': 5,
                'border-color': '#b45309',
            },
        },
        {
            selector: 'edge',
            style: {
                'curve-style': 'bezier',
                'target-arrow-shape': 'triangle',
                'arrow-scale': 1.1,
                'line-color': '#cbd5e1',
                'target-arrow-color': '#cbd5e1',
                'width': 2.5,
                'label': 'data(label)',
                'font-size': 9,
                'color': '#475569',
                'text-rotation': 'autorotate',
                'text-background-color': '#ffffff',
                'text-background-opacity': 0.85,
                'text-background-padding': 2,
                'text-background-shape': 'round-rectangle',
            },
        },
        {
            selector: 'edge[kind = "grip"]',
            style: { 'line-color': '#047857', 'target-arrow-color': '#047857', 'width': 4 },
        },
        {
            selector: 'edge[kind = "release"]',
            style: { 'line-color': '#b45309', 'target-arrow-color': '#b45309', 'width': 4 },
        },
        {
            selector: 'edge.traversing',
            style: {
                'line-color': '#0ea5e9',
                'target-arrow-color': '#0ea5e9',
                'width': 6,
                'z-index': 9999,
            },
        },
        {
            selector: 'edge.selected-edge',
            style: { 'line-color': '#0369a1', 'target-arrow-color': '#0369a1', 'width': 5 },
        },
        {
            selector: 'node.draw-source',
            style: { 'border-width': 5, 'border-color': '#0ea5e9' },
        },
        {
            selector: 'node.draw-target',
            style: { 'border-width': 5, 'border-color': '#7c3aed' },
        },
    ];

    // Grid layout over the VISIBLE elements, sorted so each station's tile
    // leads its (shown) member nodes. Run after every collapse/expand and add.
    function gridLayout() {
        return {
            name: 'grid',
            avoidOverlap: true,
            nodeDimensionsIncludeLabels: true,
            condense: false,
            padding: 24,
            sort: function (a, b) {
                var sa = a.data('station') || '', sb = b.data('station') || '';
                if (sa !== sb) return sa < sb ? -1 : 1;
                var ga = a.data('isGroup') ? 0 : 1, gb = b.data('isGroup') ? 0 : 1;
                if (ga !== gb) return ga - gb;  // station tile first
                return a.id() < b.id() ? -1 : a.id() > b.id() ? 1 : 0;
            },
        };
    }

    // Show member nodes only for expanded stations; update each tile's caret +
    // count. Then lay out just the visible elements so collapsed stations take
    // a single tile of space.
    function applyGroupVisibility() {
        if (!cy) return;
        cy.batch(function () {
            cy.nodes().forEach(function (n) {
                var s = n.data('station');
                if (n.data('isGroup')) {
                    var exp = !!expanded[s];
                    n.data('label', (exp ? '▾ ' : '▸ ') + s + (exp ? '' : ' (' + (n.data('count') || 0) + ')'));
                } else {
                    // station-less nodes always show; otherwise follow expansion
                    n.style('display', (!s || expanded[s]) ? 'element' : 'none');
                }
            });
        });
        cy.elements(':visible').layout(gridLayout()).run();
        cy.fit(undefined, 30);
    }

    function renderGraph(data) {
        var elements = buildElements(data);
        if (!elements.length) {
            showMessage('Graph loaded but contains no nodes.');
            return;
        }
        clearMessage();
        cy = cytoscape({
            container: document.getElementById('cy'),
            elements: elements,
            style: CY_STYLE,
            wheelSensitivity: 0.2,
        });
        onGraphRendered(cy);
        applyGroupVisibility();  // start collapsed + initial layout
    }

    // ── Phase B: edge editing (mode/speed) behind a claim ────────────

    var CLAIM_OWNER = 'human@xarm-graph';
    var claimSessionId =
        (window.crypto && crypto.randomUUID && crypto.randomUUID()) ||
        ('xarm-graph-' + Date.now() + '-' + Math.random().toString(16).slice(2));
    // updateClaimIndicator reads this to mark the holder "(you)".
    window.__graphClaim = { sessionId: claimSessionId };
    var claimToken = null;
    var claimHeartbeatTimer = null;

    var editingEdge = null;          // cytoscape edge currently in the panel
    var panel = document.getElementById('edge-panel');
    var edgeFromEl = document.getElementById('edge-from');
    var edgeToEl = document.getElementById('edge-to');
    var edgeModeEl = document.getElementById('edge-mode');
    var edgeSpeedEl = document.getElementById('edge-speed');
    var edgeSaveBtn = document.getElementById('edge-save-btn');
    var edgeCancelBtn = document.getElementById('edge-cancel-btn');
    var edgeErrEl = document.getElementById('edge-panel-error');
    var edgePrecosEl = document.getElementById('edge-preconditions');

    // Bind edge taps once the canvas exists. Tapping empty space closes
    // the panel; tapping an edge opens it on that edge.
    function onGraphRendered(cyInstance) {
        cyInstance.on('tap', 'edge', function (evt) { openEdgePanel(evt.target); });
        cyInstance.on('tap', function (evt) {
            if (evt.target === cyInstance) closeEdgePanel();
        });
        // Manual show/hide of a station by tapping its tile — disabled while a
        // workflow holds control (automation in progress).
        cyInstance.on('tap', 'node', function (evt) {
            var n = evt.target;
            if (!n.data('isGroup')) {
                // Member node: in draw mode it's an edge endpoint, else ignore.
                if (drawMode) pickDrawNode(n);
                return;
            }
            if (automationActive()) {
                showMessage('Locked: a workflow holds control — expand/collapse is disabled while automation is running.');
                return;
            }
            clearMessage();
            var s = n.data('station');
            if (expanded[s]) delete expanded[s]; else expanded[s] = true;
            applyGroupVisibility();
        });
    }

    // "Automation" = the device claim is held by someone other than this page.
    function automationActive() {
        return !!(latestClaimedBy && latestClaimedBy.session_id !== claimSessionId);
    }

    function showEdgeError(text) {
        if (!edgeErrEl) return;
        edgeErrEl.textContent = text;
        edgeErrEl.hidden = false;
    }
    function clearEdgeError() {
        if (edgeErrEl) edgeErrEl.hidden = true;
    }

    function openEdgePanel(edge) {
        if (!panel) return;
        if (editingEdge) editingEdge.removeClass('selected-edge');
        editingEdge = edge;
        edge.addClass('selected-edge');
        var d = edge.data();
        if (edgeFromEl) edgeFromEl.textContent = d.source;
        if (edgeToEl) edgeToEl.textContent = d.target;
        if (edgeModeEl) edgeModeEl.value = d.mode;
        if (edgeSpeedEl) edgeSpeedEl.value = (d.speed != null ? d.speed : '');
        if (edgePrecosEl) {
            var precos = d.preconditions || [];
            edgePrecosEl.textContent = precos.length ? precos.join(', ') : 'none';
        }
        clearEdgeError();
        panel.hidden = false;
    }

    function closeEdgePanel() {
        if (editingEdge) { editingEdge.removeClass('selected-edge'); editingEdge = null; }
        if (panel) panel.hidden = true;
    }

    // Acquire a claim on first edit and keep it (heartbeated) for the
    // session. Resolves to true when the claim is held, false otherwise
    // (and surfaces the reason in the edit panel).
    function ensureClaim() {
        if (claimToken) return Promise.resolve(true);
        return fetch(API_BASE + '/control/claim', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ owner: CLAIM_OWNER, session_id: claimSessionId, ttl_s: 30 }),
        }).then(function (resp) {
            if (resp.status === 200) {
                return resp.json().then(function (data) {
                    claimToken = data.claim_token;
                    startClaimHeartbeat(data.heartbeat_interval_s);
                    updateClaimIndicator({ owner: CLAIM_OWNER, session_id: claimSessionId });
                    return true;
                });
            }
            if (resp.status === 409 || resp.status === 423) {
                return resp.json().catch(function () { return {}; }).then(function (d) {
                    var owner = (d.claimed_by && d.claimed_by.owner) ||
                        (d.detail && d.detail.claimed_by && d.detail.claimed_by.owner);
                    showEdgeError('Device is controlled by ' + (owner || 'another session') + '. Try again later.');
                    return false;
                });
            }
            // 400 "connect first" — the arm isn't connected, so no claim.
            return resp.json().catch(function () { return {}; }).then(function (d) {
                var msg = (d && (d.detail || d.error)) || ('HTTP ' + resp.status);
                showEdgeError('Cannot take control: ' + msg);
                return false;
            });
        }).catch(function (e) {
            showEdgeError('Cannot take control: ' + e.message);
            return false;
        });
    }

    function startClaimHeartbeat(intervalSeconds) {
        stopClaimHeartbeat();
        var everyMs = Math.max(2000, ((intervalSeconds || 10) * 1000) / 2);
        claimHeartbeatTimer = setInterval(function () {
            if (!claimToken) return;
            fetch(API_BASE + '/control/heartbeat', {
                method: 'POST', headers: { 'X-Claim-Token': claimToken },
            }).then(function (r) {
                if (r.status === 401 || r.status === 404) handleClaimLost();
            }).catch(function () { /* transient; next beat retries */ });
        }, everyMs);
    }

    function stopClaimHeartbeat() {
        if (claimHeartbeatTimer) { clearInterval(claimHeartbeatTimer); claimHeartbeatTimer = null; }
    }

    function handleClaimLost() {
        if (claimToken === null) return;
        claimToken = null;
        stopClaimHeartbeat();
        showEdgeError('Lost control — the claim is no longer held. Save again to re-acquire.');
    }

    function releaseClaim(viaUnload) {
        var token = claimToken;
        stopClaimHeartbeat();
        claimToken = null;
        if (!token) return;
        try {
            fetch(API_BASE + '/control/release', {
                method: 'POST', headers: { 'X-Claim-Token': token }, keepalive: !!viaUnload,
            });
        } catch (e) { /* best-effort */ }
    }

    function saveEdge() {
        if (!editingEdge) return;
        var d = editingEdge.data();
        var mode = edgeModeEl ? edgeModeEl.value : d.mode;
        var speedRaw = edgeSpeedEl ? edgeSpeedEl.value : '';
        var body = { from_node: d.source, to_node: d.target, mode: mode };
        if (speedRaw !== '' && speedRaw != null) {
            var speed = parseFloat(speedRaw);
            if (isNaN(speed) || speed <= 0) {
                showEdgeError('Speed must be a number greater than 0.');
                return;
            }
            body.speed = speed;
        }
        clearEdgeError();
        if (edgeSaveBtn) edgeSaveBtn.disabled = true;
        ensureClaim().then(function (held) {
            if (!held) { if (edgeSaveBtn) edgeSaveBtn.disabled = false; return; }
            return fetch(API_BASE + '/control/graph/edge', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json', 'X-Claim-Token': claimToken },
                body: JSON.stringify(body),
            }).then(function (resp) {
                if (resp.status === 200) {
                    return resp.json().then(function (data) {
                        var u = data.updated || {};
                        // Update the in-memory edge + label in place (no rebuild).
                        // Preconditions are unchanged by this edit but kept in
                        // the recomputed label so they don't disappear on save.
                        editingEdge.data('mode', u.mode);
                        editingEdge.data('speed', u.speed);
                        editingEdge.data('label', edgeLabel(u.mode, u.speed, editingEdge.data('preconditions')));
                        closeEdgePanel();
                    });
                }
                if (resp.status === 423) {
                    handleClaimLost();
                    return resp.json().catch(function () { return {}; }).then(function (d2) {
                        var owner = d2.detail && d2.detail.claimed_by && d2.detail.claimed_by.owner;
                        showEdgeError(owner
                            ? 'Locked: control held by ' + owner + '.'
                            : 'Locked: control is held elsewhere.');
                    });
                }
                return resp.json().catch(function () { return {}; }).then(function (d2) {
                    var msg = (d2 && (d2.detail || d2.error)) || ('HTTP ' + resp.status);
                    showEdgeError('Save failed: ' + (typeof msg === 'string' ? msg : JSON.stringify(msg)));
                });
            });
        }).catch(function (e) {
            showEdgeError('Save failed: ' + e.message);
        }).then(function () {
            if (edgeSaveBtn) edgeSaveBtn.disabled = false;
        });
    }

    if (edgeSaveBtn) edgeSaveBtn.addEventListener('click', saveEdge);
    if (edgeCancelBtn) edgeCancelBtn.addEventListener('click', closeEdgePanel);
    window.addEventListener('beforeunload', function () {
        if (claimToken) releaseClaim(true);
    });

    // ── Add node ─────────────────────────────────────────────────────

    var nodeArmEl = document.getElementById('node-arm');
    var nodeRailEl = document.getElementById('node-rail');
    var nodeGripperEl = document.getElementById('node-gripper');
    var nodePayloadEl = document.getElementById('node-payload');
    var nodeIdEl = document.getElementById('node-id');
    var nodeTagsEl = document.getElementById('node-tags');
    var nodeAddBtn = document.getElementById('node-add-btn');
    var addNodeErrEl = document.getElementById('add-node-error');

    function fillSelect(sel, values) {
        if (!sel) return;
        sel.innerHTML = values.map(function (v) {
            return '<option value="' + v + '">' + v + '</option>';
        }).join('');
    }
    function showAddNodeError(text) {
        if (!addNodeErrEl) return;
        addNodeErrEl.textContent = text;
        addNodeErrEl.hidden = false;
    }
    function clearAddNodeError() { if (addNodeErrEl) addNodeErrEl.hidden = true; }

    // Populate the form's dropdowns: gripper/payload from the /graph
    // catalogs, arm poses from /locations, rail from /track/locations.
    function populateAddNodeForm(graphData) {
        (graphData.gripper_states || []).forEach(function (g) { gripperStrokes[g.name] = g.stroke; });
        fillSelect(nodeGripperEl, (graphData.gripper_states || []).map(function (g) { return g.name; }));
        fillSelect(nodePayloadEl, (graphData.payloads || []).map(function (p) { return p.name; }));
        fetch(API_BASE + '/locations').then(function (r) { return r.json(); })
            .then(function (d) { fillSelect(nodeArmEl, (d && d.locations) || []); })
            .catch(function () {});
        fetch(API_BASE + '/track/locations').then(function (r) { return r.json(); })
            .then(function (d) { fillSelect(nodeRailEl, (d && d.locations) || []); })
            .catch(function () {});
    }

    // Suggest the node id from the pose name when the id box is empty, so
    // the convention (id == pose, + _empty/_held) is the path of least effort.
    if (nodeArmEl) {
        nodeArmEl.addEventListener('change', function () {
            if (nodeIdEl && !nodeIdEl.value.trim()) nodeIdEl.value = nodeArmEl.value;
        });
    }

    function addNode() {
        var id = (nodeIdEl && nodeIdEl.value.trim()) || '';
        var arm = nodeArmEl && nodeArmEl.value;
        var rail = nodeRailEl && nodeRailEl.value;
        var gripper = nodeGripperEl && nodeGripperEl.value;
        var payload = nodePayloadEl && nodePayloadEl.value;
        if (!id) { showAddNodeError('Node id is required.'); return; }
        if (!arm || !rail || !gripper || !payload) {
            showAddNodeError('Pose, rail, gripper and payload are all required.');
            return;
        }
        if (cy && cy.getElementById(id).nonempty()) {
            showAddNodeError('A node with id "' + id + '" already exists.');
            return;
        }
        var tags = (nodeTagsEl && nodeTagsEl.value.trim())
            ? nodeTagsEl.value.split(',').map(function (t) { return t.trim(); }).filter(Boolean)
            : null;
        var body = { id: id, arm: arm, rail: rail, gripper: gripper, payload: payload, tags: tags };
        clearAddNodeError();
        if (nodeAddBtn) nodeAddBtn.disabled = true;
        ensureClaim().then(function (held) {
            // ensureClaim surfaces its reason in the edge panel; mirror it here.
            if (!held) {
                if (edgeErrEl && !edgeErrEl.hidden) showAddNodeError(edgeErrEl.textContent);
                if (nodeAddBtn) nodeAddBtn.disabled = false;
                return;
            }
            return fetch(API_BASE + '/control/graph/node', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json', 'X-Claim-Token': claimToken },
                body: JSON.stringify(body),
            }).then(function (resp) {
                if (resp.status === 200) {
                    return resp.json().then(function (data) {
                        addNodeToCanvas(data.created);
                        if (nodeIdEl) nodeIdEl.value = '';
                        if (nodeTagsEl) nodeTagsEl.value = '';
                    });
                }
                if (resp.status === 423) {
                    handleClaimLost();
                    showAddNodeError('Locked: control is held elsewhere.');
                    return;
                }
                return resp.json().catch(function () { return {}; }).then(function (d2) {
                    var msg = (d2 && (d2.detail || d2.error)) || ('HTTP ' + resp.status);
                    showAddNodeError('Add failed: ' + (typeof msg === 'string' ? msg : JSON.stringify(msg)));
                });
            });
        }).catch(function (e) {
            showAddNodeError('Add failed: ' + e.message);
        }).then(function () {
            if (nodeAddBtn) nodeAddBtn.disabled = false;
        });
    }

    function addNodeToCanvas(created) {
        if (!cy || !created) return;
        var nd = nodeData({
            id: created.id, arm: created.arm, rail: created.rail,
            gripper: created.gripper, payload: created.payload, tags: created.tags || [],
        });
        var station = nd.station;
        if (station) {
            var grp = cy.getElementById('grp:' + station);
            if (grp.empty()) {
                // New station — create its tile.
                cy.add({ group: 'nodes', data: {
                    id: 'grp:' + station, label: '▾ ' + station, isGroup: true,
                    station: station, color: colorForStation(station), count: 1,
                } });
            } else {
                grp.data('count', (grp.data('count') || 0) + 1);
            }
            expanded[station] = true;  // reveal the station so the new node shows
        }
        cy.add({ group: 'nodes', data: nd });
        applyGroupVisibility();
    }

    if (nodeAddBtn) nodeAddBtn.addEventListener('click', addNode);

    // ── Draw edge (pick source → pick target → create) ───────────────

    var drawMode = false;
    var drawFrom = null, drawTo = null;
    var drawToggleBtn = document.getElementById('edge-draw-toggle');
    var drawFromEl = document.getElementById('draw-from');
    var drawToEl = document.getElementById('draw-to');
    var drawFormEl = document.getElementById('edge-draw-form');
    var drawModeEl = document.getElementById('draw-mode');
    var drawSpeedEl = document.getElementById('draw-speed');
    var drawGripGroup = document.getElementById('draw-grip-group');
    var drawGripStrokeEl = document.getElementById('draw-grip-stroke');
    var drawGripForceEl = document.getElementById('draw-grip-force');
    var drawReleaseGroup = document.getElementById('draw-release-group');
    var drawReleaseStrokeEl = document.getElementById('draw-release-stroke');
    var drawCreateBtn = document.getElementById('draw-create-btn');
    var drawCancelBtn = document.getElementById('draw-cancel-btn');
    var drawErrEl = document.getElementById('edge-draw-error');
    var drawIdentitySwap = false;

    function showDrawError(t) { if (drawErrEl) { drawErrEl.textContent = t; drawErrEl.hidden = false; } }
    function clearDrawError() { if (drawErrEl) drawErrEl.hidden = true; }

    function setDrawMode(on) {
        drawMode = on;
        if (drawToggleBtn) {
            drawToggleBtn.textContent = on ? 'Stop drawing' : 'Pick source…';
            drawToggleBtn.classList.toggle('is-active', on);
        }
        if (!on) clearDrawSelection();
    }

    function clearDrawSelection() {
        if (cy) cy.nodes().removeClass('draw-source draw-target');
        drawFrom = drawTo = null;
        if (drawFromEl) drawFromEl.textContent = '—';
        if (drawToEl) drawToEl.textContent = '—';
        if (drawFormEl) drawFormEl.hidden = true;
        clearDrawError();
    }

    function pickDrawNode(n) {
        if (!drawFrom) {
            drawFrom = n.id();
            n.addClass('draw-source');
            if (drawFromEl) drawFromEl.textContent = drawFrom;
            clearDrawError();
        } else if (!drawTo && n.id() !== drawFrom) {
            drawTo = n.id();
            n.addClass('draw-target');
            if (drawToEl) drawToEl.textContent = drawTo;
            openDrawForm();
        }
    }

    // Show the create form; reveal grip/release inputs based on the payload
    // change implied by the two endpoints (mirrors the loader's rules).
    function openDrawForm() {
        var fp = (cy.getElementById(drawFrom).data('payload')) || 'empty';
        var tp = (cy.getElementById(drawTo).data('payload')) || 'empty';
        var gripNeeded = fp === 'empty' && tp !== 'empty';
        var releaseNeeded = fp !== 'empty' && tp === 'empty';
        drawIdentitySwap = fp !== 'empty' && tp !== 'empty' && fp !== tp;
        if (drawGripGroup) drawGripGroup.hidden = !gripNeeded;
        if (drawReleaseGroup) drawReleaseGroup.hidden = !releaseNeeded;
        if (gripNeeded && drawGripStrokeEl && !drawGripStrokeEl.value) {
            drawGripStrokeEl.value = gripperStrokes.grip_plate || 100;
        }
        clearDrawError();
        if (drawIdentitySwap) {
            showDrawError('Cannot connect two different non-empty payloads directly — route through an empty state.');
        }
        if (drawFormEl) drawFormEl.hidden = false;
    }

    function createEdge() {
        if (!drawFrom || !drawTo) { showDrawError('Pick a source and a target node first.'); return; }
        if (drawIdentitySwap) { showDrawError('Cannot connect two different non-empty payloads directly.'); return; }
        var body = { from_node: drawFrom, to_node: drawTo, mode: drawModeEl ? drawModeEl.value : 'joint' };
        var speedRaw = drawSpeedEl ? drawSpeedEl.value : '';
        if (speedRaw !== '' && speedRaw != null) {
            var sp = parseFloat(speedRaw);
            if (isNaN(sp) || sp <= 0) { showDrawError('Speed must be a number greater than 0.'); return; }
            body.speed = sp;
        }
        if (drawGripGroup && !drawGripGroup.hidden) {
            var gs = parseFloat(drawGripStrokeEl && drawGripStrokeEl.value);
            if (isNaN(gs)) { showDrawError('Grip stroke is required for an empty → held edge.'); return; }
            body.grip = { stroke: gs };
            var gf = parseFloat(drawGripForceEl && drawGripForceEl.value);
            if (!isNaN(gf)) body.grip.force = gf;
        }
        if (drawReleaseGroup && !drawReleaseGroup.hidden) {
            var rs = parseFloat(drawReleaseStrokeEl && drawReleaseStrokeEl.value);
            body.release = isNaN(rs) ? {} : { stroke: rs };
        }
        clearDrawError();
        if (drawCreateBtn) drawCreateBtn.disabled = true;
        ensureClaim().then(function (held) {
            if (!held) {
                if (edgeErrEl && !edgeErrEl.hidden) showDrawError(edgeErrEl.textContent);
                if (drawCreateBtn) drawCreateBtn.disabled = false;
                return;
            }
            return fetch(API_BASE + '/control/graph/edge/create', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json', 'X-Claim-Token': claimToken },
                body: JSON.stringify(body),
            }).then(function (resp) {
                if (resp.status === 200) {
                    return resp.json().then(function (data) {
                        addEdgeToCanvas(data.created);
                        clearDrawSelection();  // ready for the next edge; stay in draw mode
                    });
                }
                if (resp.status === 423) {
                    handleClaimLost();
                    showDrawError('Locked: control is held elsewhere.');
                    return;
                }
                return resp.json().catch(function () { return {}; }).then(function (d2) {
                    var msg = (d2 && (d2.detail || d2.error)) || ('HTTP ' + resp.status);
                    showDrawError('Create failed: ' + (typeof msg === 'string' ? msg : JSON.stringify(msg)));
                });
            });
        }).catch(function (e) {
            showDrawError('Create failed: ' + e.message);
        }).then(function () {
            if (drawCreateBtn) drawCreateBtn.disabled = false;
        });
    }

    function addEdgeToCanvas(created) {
        if (!cy || !created) return;
        var kind = created.grips ? 'grip' : (created.releases ? 'release' : 'plain');
        cy.add({ group: 'edges', data: {
            id: 'e:' + created.from + '->' + created.to,
            source: created.from, target: created.to,
            label: edgeLabel(created.mode, created.speed, created.preconditions || []),
            kind: kind, mode: created.mode, speed: created.speed,
            preconditions: created.preconditions || [],
        } });
        applyGroupVisibility();
    }

    if (drawToggleBtn) drawToggleBtn.addEventListener('click', function () { setDrawMode(!drawMode); });
    if (drawCreateBtn) drawCreateBtn.addEventListener('click', createEdge);
    if (drawCancelBtn) drawCancelBtn.addEventListener('click', clearDrawSelection);

    // ── Live state ───────────────────────────────────────────────────

    function applyLiveState(live) {
        if (modeEl) modeEl.textContent = live.graph_mode || 'off';
        var node = live.current_node || null;
        if (currentEl) currentEl.textContent = node || '—';
        var payload = live.declared_payload || 'empty';
        if (payloadEl) payloadEl.textContent = payload;

        if (!cy) return;
        var holding = node && payload && payload !== 'empty';

        // Auto-follow: expand the station the robot is in, collapse the one it
        // left. Runs on station change only (not every tick).
        var station = null;
        if (node) {
            var ne = cy.getElementById(node);
            if (ne && ne.nonempty()) station = ne.data('station');
        }
        if (station !== autoStation) {
            if (autoStation) delete expanded[autoStation];
            if (station) expanded[station] = true;
            autoStation = station;
            applyGroupVisibility();
        }

        // Flash the edge being traversed when the pinned node changes.
        if (node && currentNodeId && node !== currentNodeId) {
            flashEdge(currentNodeId, node);
        }

        cy.nodes().removeClass('current holding');
        if (node) {
            var ele = cy.getElementById(node);
            if (ele && ele.nonempty()) {
                ele.addClass('current');
                if (holding) ele.addClass('holding');
            }
        }
        currentNodeId = node;
    }

    function flashEdge(fromId, toId) {
        var edge = cy.getElementById('e:' + fromId + '->' + toId);
        if (!edge || edge.empty()) return;
        edge.addClass('traversing');
        if (traverseTimer) clearTimeout(traverseTimer);
        traverseTimer = setTimeout(function () {
            edge.removeClass('traversing');
        }, 1200);
    }

    function updateClaimIndicator(claimedBy) {
        latestClaimedBy = claimedBy || null;  // drives automationActive()
        if (!claimEl) return;
        claimEl.classList.remove('claim-free', 'claim-mine', 'claim-other');
        if (!claimedBy) {
            claimEl.textContent = 'Control: nobody (open)';
            claimEl.classList.add('claim-free');
            return;
        }
        // Phase A holds no claim, so the holder is always "other" from here.
        var mine = window.__graphClaim && claimedBy.session_id === window.__graphClaim.sessionId;
        claimEl.textContent = 'Control: ' + claimedBy.owner + (mine ? ' (you)' : '');
        claimEl.classList.add(mine ? 'claim-mine' : 'claim-other');
    }

    // ── Data loading ─────────────────────────────────────────────────

    function fetchGraph() {
        return fetch(API_BASE + '/graph').then(function (resp) {
            if (resp.status === 404) {
                throw new Error('no_graph');
            }
            if (!resp.ok) {
                throw new Error('HTTP ' + resp.status);
            }
            return resp.json();
        });
    }

    function initialLoad() {
        if (typeof window.cytoscape === 'undefined') {
            showMessage('Cytoscape failed to load (cytoscape.min.js missing).');
            return;
        }
        fetchGraph().then(function (data) {
            renderGraph(data);
            applyLiveState(data);
            populateAddNodeForm(data);
        }).catch(function (err) {
            if (err.message === 'no_graph') {
                showMessage('No motion graph loaded (motion_graph.yaml missing or invalid).');
            } else {
                showMessage('Failed to load graph: ' + err.message);
            }
        });
    }

    // Fallback live-state poll — only runs when the WS push has gone stale.
    function pollLiveState() {
        if (Date.now() - lastPushAt < PUSH_STALE_MS) return;
        if (!cy) return;
        fetchGraph().then(function (data) {
            applyLiveState(data);
        }).catch(function () { /* transient; next tick retries */ });
    }

    // ── WebSocket ─────────────────────────────────────────────────────

    function connectWebSocket() {
        var socket;
        try {
            socket = new WebSocket(WS_URL);
        } catch (e) {
            return;
        }
        socket.onmessage = function (event) {
            var message;
            try { message = JSON.parse(event.data); } catch (e) { return; }
            if (message.type !== 'status_update') return;
            lastPushAt = Date.now();
            var details = (message.data && message.data.details) || {};
            var mg = details.motion_graph;
            if (mg) {
                applyLiveState({
                    graph_mode: mg.graph_mode,
                    current_node: mg.current_node,
                    declared_payload: mg.declared_payload,
                });
            }
            updateClaimIndicator(details.claimed_by || null);
        };
        socket.onclose = function () {
            // Reconnect with a short backoff; the fallback poll covers the gap.
            setTimeout(connectWebSocket, 2000);
        };
        socket.onerror = function () { try { socket.close(); } catch (e) {} };
    }

    // ── Boot ──────────────────────────────────────────────────────────

    document.addEventListener('DOMContentLoaded', function () {
        initialLoad();
        connectWebSocket();
        setInterval(pollLiveState, 1500);
    });

    // Expose hooks so Phase B can extend without rewriting Phase A.
    window.__graphViewer = {
        getCy: function () { return cy; },
        fetchGraph: fetchGraph,
        applyLiveState: applyLiveState,
        apiBase: API_BASE,
    };
})();
