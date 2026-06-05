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

    function buildElements(data) {
        var elements = [];
        (data.nodes || []).forEach(function (n) {
            var station = (n.tags && n.tags.length) ? n.tags[0] : '';
            var held = n.payload && n.payload !== 'empty';
            elements.push({
                group: 'nodes',
                data: {
                    id: n.id,
                    // ▣ marks topology nodes whose state holds labware, so the
                    // pick/place ladder is legible even before the arm moves.
                    label: held ? n.id + ' ▣' : n.id,
                    station: station,
                    color: colorForStation(station),
                    rail: n.rail,
                    payload: n.payload,
                },
            });
        });
        (data.edges || []).forEach(function (e) {
            var kind = e.grips ? 'grip' : (e.releases ? 'release' : 'plain');
            var label = e.mode + (e.speed != null ? ' @ ' + e.speed : '');
            elements.push({
                group: 'edges',
                data: {
                    id: 'e:' + e.from + '->' + e.to,
                    source: e.from,
                    target: e.to,
                    label: label,
                    kind: kind,
                    mode: e.mode,
                    speed: e.speed,
                },
            });
        });
        return elements;
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
            selector: 'edge.grip',
            style: { 'line-color': '#047857', 'target-arrow-color': '#047857', 'width': 4 },
        },
        {
            selector: 'edge.release',
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
    ];

    function renderGraph(data) {
        var elements = buildElements(data);
        if (!elements.length) {
            showMessage('Graph loaded but contains no nodes.');
            return;
        }
        clearMessage();
        var hasHome = (data.nodes || []).some(function (n) { return n.id === 'home'; });
        cy = cytoscape({
            container: document.getElementById('cy'),
            elements: elements,
            style: CY_STYLE,
            layout: {
                name: 'breadthfirst',
                directed: true,
                roots: hasHome ? '#home' : undefined,
                spacingFactor: 1.3,
                padding: 24,
            },
            wheelSensitivity: 0.2,
        });
        onGraphRendered(cy);
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

    // Bind edge taps once the canvas exists. Tapping empty space closes
    // the panel; tapping an edge opens it on that edge.
    function onGraphRendered(cyInstance) {
        cyInstance.on('tap', 'edge', function (evt) { openEdgePanel(evt.target); });
        cyInstance.on('tap', function (evt) {
            if (evt.target === cyInstance) closeEdgePanel();
        });
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
                        editingEdge.data('mode', u.mode);
                        editingEdge.data('speed', u.speed);
                        editingEdge.data('label', u.mode + (u.speed != null ? ' @ ' + u.speed : ''));
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

    // ── Live state ───────────────────────────────────────────────────

    function applyLiveState(live) {
        if (modeEl) modeEl.textContent = live.graph_mode || 'off';
        var node = live.current_node || null;
        if (currentEl) currentEl.textContent = node || '—';
        var payload = live.declared_payload || 'empty';
        if (payloadEl) payloadEl.textContent = payload;

        if (!cy) return;
        var holding = node && payload && payload !== 'empty';

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
