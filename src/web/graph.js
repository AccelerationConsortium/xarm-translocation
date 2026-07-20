/* Motion Graph viewer (Phase A: read-only render + live state).
 *
 * Fetches GET /graph (proxied through the web server to the API on :8000),
 * renders the node/edge topology with Cytoscape, and tracks where the arm
 * is live by subscribing to the same /ws status stream the control panel
 * uses. No claim and no writes in Phase A. */
(function () {
    'use strict';

    // HTTP goes through the page's own origin (the web server proxies /graph
    // and /control to the API). Base-path prefix the page is served under:
    // "" direct (…/web/…), or "/xarm5" behind the single Caddy edge
    // (…/xarm5/web/…) — API + WS must carry it (docs/SINGLE_EDGE_SSO_PLAN.md).
    // The WebSocket can't be proxied by the legacy :6001 http.server, so there
    // it connects straight to the API on :8000 — mirroring main.js's logic.
    var _pathname = window.location.pathname;
    var _webIdx = _pathname.indexOf('/web');
    var BASE_PATH = _webIdx > 0 ? _pathname.slice(0, _webIdx) : '';
    var API_BASE = window.location.protocol + '//' + window.location.host + BASE_PATH;
    var wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    var wsHost = BASE_PATH
        ? window.location.host
        : (window.location.port === '8000'
            ? window.location.host
            : window.location.hostname + ':8000');
    var WS_URL = wsProtocol + '://' + wsHost + BASE_PATH + '/ws';

    // Live-push freshness: while WS pushes arrive the fallback HTTP poll for
    // live state stays idle (it only fires when the socket goes quiet).
    var PUSH_STALE_MS = 2000;
    var lastPushAt = 0;

    var cy = null;
    var gripperCatalog = {};    // gripper_state name -> {stroke, intent} (from /graph)

    // Persisted layout — node positions, which stations are open, and the
    // pan/zoom — kept in localStorage so a save (add node/edge) or a reopen
    // restores the exact arrangement instead of re-gridding from scratch.
    var LAYOUT_KEY = 'xarm-graph-layout-v2';
    var savedLayout = loadSavedLayout();          // {positions, expanded, pan, zoom}
    var expanded = savedLayout.expanded || {};    // station -> true when its nodes are shown
    var placedIds = {};         // node ids that already have a committed position
    var saveTimer = null;       // debounce handle for persistLayoutSoon
    var stationAnchors = {};    // station -> its anchor ("home") node id
    // While a home node is dragged, its placed siblings move with it (the whole
    // submap translates together). Captured at grab, applied on each drag tick.
    var dragHome = null, dragHomeStart = null, dragSiblings = null;

    var autoStation = null;     // station auto-expanded to follow the robot
    var latestClaimedBy = null; // device claim holder from /status (for the lock)
    var currentNodeId = null;   // last applied live current_node
    var traverseTimer = null;

    function loadSavedLayout() {
        try {
            var raw = window.localStorage.getItem(LAYOUT_KEY);
            var obj = raw ? JSON.parse(raw) : null;
            return (obj && typeof obj === 'object') ? obj : {};
        } catch (e) { return {}; }
    }

    // Write the live arrangement back to localStorage. Only nodes we've
    // actually positioned (placedIds) are saved, so a never-expanded
    // station's members stay unplaced and fan out fresh when first opened.
    function persistLayout() {
        if (!cy) return;
        var positions = {};
        cy.nodes().forEach(function (n) {
            if (!placedIds[n.id()]) return;
            var p = n.position();
            if (p && isFinite(p.x) && isFinite(p.y)) positions[n.id()] = { x: p.x, y: p.y };
        });
        savedLayout.positions = positions;
        savedLayout.expanded = expanded;
        savedLayout.pan = cy.pan();
        savedLayout.zoom = cy.zoom();
        // localStorage is an offline cache; the device file is the source of
        // truth shared across every PC that connects.
        try { window.localStorage.setItem(LAYOUT_KEY, JSON.stringify(savedLayout)); } catch (e) {}
        try {
            fetch(API_BASE + '/graph/layout', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    positions: positions, expanded: expanded,
                    pan: savedLayout.pan, zoom: savedLayout.zoom,
                }),
            }).catch(function () { /* offline: localStorage still holds it */ });
        } catch (e) { /* fetch unavailable */ }
    }

    // The device-stored geometry shared across PCs. Resolves to null on any
    // failure so the caller falls back to the localStorage copy.
    function fetchLayout() {
        return fetch(API_BASE + '/graph/layout')
            .then(function (r) { return r.ok ? r.json() : null; })
            .catch(function () { return null; });
    }

    // Debounced save for high-frequency events (drag, pan, zoom).
    function persistLayoutSoon() {
        if (saveTimer) clearTimeout(saveTimer);
        saveTimer = setTimeout(persistLayout, 300);
    }

    var modeEl = document.getElementById('graph-mode');
    var currentEl = document.getElementById('graph-current');
    var gripperEl = document.getElementById('graph-gripper');
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

    // Compact badge for one gripper leaf: ▢ empty, ▣ grasp states,
    // ◇ position (reach) states.
    function leafGlyph(stateName) {
        if (stateName === 'empty') return '▢';
        var spec = gripperCatalog[stateName] || {};
        return spec.intent === 'position' ? '◇' : '▣';
    }

    // Cytoscape node data for one graph node. A node is an arm position;
    // its gripper leaves render as compact badges after the id (e.g.
    // "deck_slot1_low ▢▣"), and ⇄ marks nodes where grip/release/narrow
    // transitions are whitelisted.
    function nodeData(n) {
        var station = (n.tags && n.tags.length) ? n.tags[0] : '';
        var states = n.gripper_states || ['empty'];
        var transitions = n.gripper_transitions || [];
        var badges = states.map(leafGlyph).join('');
        var base = n.id + ' ' + badges + (transitions.length ? ' ⇄' : '');
        return {
            id: n.id,
            base: base,
            label: base,
            station: station,
            color: colorForStation(station),
            rail: n.rail,
            gripper_states: states,
            gripper_transitions: transitions,
        };
    }

    // Pick a station's anchor ("home") node — the one that stays visible when
    // the station is collapsed and that the rest fan out from. Prefer a node
    // tagged `global_home`, then `<station>_home`, then any `*_home`, then the
    // lowest id (deterministic so the anchor never jumps between loads).
    function chooseAnchor(station, members) {
        var tagged = members.filter(function (m) { return (m.tags || []).indexOf('global_home') !== -1; });
        if (tagged.length) return tagged[0].id;
        var exact = members.filter(function (m) { return m.id === station + '_home'; });
        if (exact.length) return exact[0].id;
        var ends = members.filter(function (m) { return /_home$/.test(m.id); });
        if (ends.length) return ends[0].id;
        return members.map(function (m) { return m.id; }).sort()[0];
    }

    function buildElements(data) {
        var elements = [];
        // The catalog drives leaf badges; capture it before nodeData runs.
        gripperCatalog = data.gripper_state_catalog || {};
        var nodes = data.nodes || [];
        // Group members by station (first tag) and elect one anchor per station.
        // No synthetic "tile" node — the station's home node IS the collapse
        // target and the origin its siblings fan out from.
        var byStation = {};
        nodes.forEach(function (n) {
            var s = (n.tags && n.tags.length) ? n.tags[0] : '';
            (byStation[s] = byStation[s] || []).push(n);
        });
        stationAnchors = {};
        Object.keys(byStation).forEach(function (s) {
            if (s) stationAnchors[s] = chooseAnchor(s, byStation[s]);
        });
        nodes.forEach(function (n) {
            var d = nodeData(n);
            if (d.station && stationAnchors[d.station] === n.id) {
                d.isAnchor = true;
                d.count = byStation[d.station].length;  // members incl. the anchor
            }
            elements.push({ group: 'nodes', data: d });
        });
        (data.edges || []).forEach(function (e) {
            var precos = e.preconditions || [];
            elements.push({
                group: 'edges',
                data: {
                    id: 'e:' + e.from + '->' + e.to,
                    source: e.from,
                    target: e.to,
                    label: edgeLabel(e.mode, e.speed, precos),
                    kind: 'plain',
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
            // Anchor ("home") node: the always-visible head of a station you
            // click to fan its siblings out / collapse them back. Heavier
            // border + bolder label so it reads as the group head.
            selector: 'node[?isAnchor]',
            style: {
                'font-size': 13,
                'font-weight': 800,
                'padding': '14px',
                'border-width': 4,
                'border-color': 'rgba(15,23,42,0.5)',
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

    // Seed anchor (home) nodes that have no position yet (first load, or a
    // brand-new station) into a simple grid, started below any anchors already
    // placed so a new station doesn't land on an existing one.
    function seedAnchorGrid(unplaced, allAnchors) {
        var SPACING = 300;
        var startX = 0, startY = 0;
        var placedAnchors = allAnchors.filter(function (n) { return placedIds[n.id()]; });
        if (placedAnchors.nonempty()) {
            var maxY = -Infinity;
            placedAnchors.forEach(function (n) { maxY = Math.max(maxY, n.position('y')); });
            startY = maxY + SPACING;
        }
        var arr = unplaced.toArray();
        var perRow = Math.max(1, Math.ceil(Math.sqrt(arr.length)));
        arr.forEach(function (n, i) {
            n.position({ x: startX + (i % perRow) * SPACING, y: startY + Math.floor(i / perRow) * SPACING });
            placedIds[n.id()] = true;
        });
    }

    // Fan a station's not-yet-placed siblings out in concentric rings around
    // its home node — so expanding grows the submap "from the home node"
    // instead of re-gridding the whole graph. Siblings the user already
    // positioned keep their spot.
    function fanAround(home, unplaced) {
        var c = home.position();
        var STEP = 150, MIN_ARC = 140;  // ring spacing / min gap between nodes
        var arr = unplaced.toArray();
        var idx = 0, ring = 1;
        while (idx < arr.length) {
            var r = STEP * ring;
            var cap = Math.max(1, Math.floor((2 * Math.PI * r) / MIN_ARC));
            for (var k = 0; k < cap && idx < arr.length; k++, idx++) {
                var ang = (2 * Math.PI * k) / cap - Math.PI / 2;
                arr[idx].position({ x: c.x + r * Math.cos(ang), y: c.y + r * Math.sin(ang) });
                placedIds[arr[idx].id()] = true;
            }
            ring += 1;
        }
    }

    // Position nodes without disturbing any the user (or a prior session)
    // already placed: restore saved spots, seed unplaced anchors, and fan each
    // expanded station's unplaced siblings around its home node. No global
    // relayout, and no auto-fit unless opts.fit is set (fresh page, nothing saved).
    function placeNodes(opts) {
        opts = opts || {};
        var positions = savedLayout.positions || {};
        cy.nodes().forEach(function (n) {
            var id = n.id();
            // Layout migration: ids collapsed from '<id>_empty' (and the
            // '<id>_grip_120' sibling) to bare '<id>' in schema 0.2 — reuse
            // the old saved spot so the operator's arrangement survives.
            var p = positions[id] || positions[id + '_empty'] || positions[id + '_grip_120'];
            if (!placedIds[id] && p) {
                n.position({ x: p.x, y: p.y });
                placedIds[id] = true;
            }
        });
        // Anchors (+ any station-less nodes) are the always-visible skeleton.
        var anchors = cy.nodes().filter(function (n) { return n.data('isAnchor') || !n.data('station'); });
        var unplacedAnchors = anchors.filter(function (n) { return !placedIds[n.id()]; });
        if (unplacedAnchors.nonempty()) seedAnchorGrid(unplacedAnchors, anchors);
        Object.keys(expanded).forEach(function (s) {
            if (!expanded[s]) return;
            var home = cy.getElementById(stationAnchors[s] || '');
            if (home.empty() || !placedIds[home.id()]) return;
            var unplaced = cy.nodes().filter(function (n) {
                return n.data('station') === s && !n.data('isAnchor') && !placedIds[n.id()];
            });
            if (unplaced.nonempty()) fanAround(home, unplaced);
        });
        if (opts.fit) cy.fit(undefined, 30);
    }

    // Anchors are always visible; their label gains a caret + the count of
    // hidden siblings when collapsed. Plain members show only when their
    // station is expanded. Positions persist, so a collapse/expand or a save
    // no longer collapses (re-grids) the view.
    function applyGroupVisibility(opts) {
        if (!cy) return;
        cy.batch(function () {
            cy.nodes().forEach(function (n) {
                var s = n.data('station');
                if (n.data('isAnchor')) {
                    var hidden = Math.max(0, (n.data('count') || 1) - 1);
                    var suffix = hidden ? (expanded[s] ? '  ▾' : '  ▸ ' + hidden) : '';
                    n.data('label', n.data('base') + suffix);
                    n.style('display', 'element');
                } else {
                    // station-less nodes always show; otherwise follow expansion
                    n.style('display', (!s || expanded[s]) ? 'element' : 'none');
                }
            });
        });
        placeNodes(opts);
        persistLayoutSoon();
    }

    // Forget the saved arrangement and lay everything out fresh (re-seed tiles,
    // re-fan open stations) — the "undo my dragging" escape hatch.
    function resetLayout() {
        try { window.localStorage.removeItem(LAYOUT_KEY); } catch (e) {}
        savedLayout = {};
        placedIds = {};
        if (cy) applyGroupVisibility({ fit: true });
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
            // Drag the empty background to pan the whole map; box-selection
            // (on by default) would otherwise hijack that drag into a marquee.
            userPanningEnabled: true,
            userZoomingEnabled: true,
            boxSelectionEnabled: false,
        });
        onGraphRendered(cy);
        // Restore the saved arrangement; only auto-fit when nothing is saved.
        var fresh = !(savedLayout.positions && Object.keys(savedLayout.positions).length);
        applyGroupVisibility({ fit: fresh });
        if (!fresh && savedLayout.zoom && savedLayout.pan) {
            cy.zoom(savedLayout.zoom);
            cy.pan(savedLayout.pan);
        }
        // Persist user-driven moves and viewport changes so they survive reopen.
        cy.on('dragfree', 'node', persistLayoutSoon);
        cy.on('pan zoom', persistLayoutSoon);

        // Drag a home node → its submap moves with it. On grab, snapshot the
        // home and every already-placed sibling; on each drag tick, shift those
        // siblings by the home's delta so relative positions are preserved.
        cy.on('grab', 'node', function (evt) {
            var n = evt.target;
            if (!n.data('isAnchor')) { dragHome = null; return; }
            var s = n.data('station');
            dragHome = n;
            dragHomeStart = { x: n.position('x'), y: n.position('y') };
            dragSiblings = [];
            cy.nodes().forEach(function (m) {
                if (m.id() !== n.id() && m.data('station') === s && placedIds[m.id()]) {
                    dragSiblings.push({ node: m, x: m.position('x'), y: m.position('y') });
                }
            });
        });
        cy.on('drag', 'node', function (evt) {
            if (!dragHome || evt.target.id() !== dragHome.id() || !dragSiblings) return;
            var dx = dragHome.position('x') - dragHomeStart.x;
            var dy = dragHome.position('y') - dragHomeStart.y;
            dragSiblings.forEach(function (it) {
                it.node.position({ x: it.x + dx, y: it.y + dy });
            });
        });
        cy.on('free', 'node', function () { dragHome = null; dragHomeStart = null; dragSiblings = null; });
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
    var edgeDeleteBtn = document.getElementById('edge-delete-btn');
    var edgeErrEl = document.getElementById('edge-panel-error');
    var edgePrecosEl = document.getElementById('edge-preconditions');

    // Bind edge taps once the canvas exists. Tapping empty space closes
    // the panel; tapping an edge opens it on that edge.
    function onGraphRendered(cyInstance) {
        cyInstance.on('tap', 'edge', function (evt) { openEdgePanel(evt.target); });
        cyInstance.on('tap', function (evt) {
            if (evt.target === cyInstance) closeEdgePanel();
        });
        // Tapping a station's home node fans its siblings out / collapses them
        // back to it. In draw mode any node (home or not) is an edge endpoint.
        // Expand/collapse is locked while a workflow holds control.
        cyInstance.on('tap', 'node', function (evt) {
            var n = evt.target;
            if (drawMode) { pickDrawNode(n); return; }
            if (!n.data('isAnchor')) return;          // plain member: nothing to toggle
            if ((n.data('count') || 1) <= 1) return;  // lone node: no siblings to show
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

    // Delete the edge currently open in the panel (claim-gated, like save).
    // On success the edge is removed from the canvas and the panel closes.
    function deleteEdge() {
        if (!editingEdge) return;
        var d = editingEdge.data();
        var body = { from_node: d.source, to_node: d.target };
        clearEdgeError();
        if (edgeDeleteBtn) edgeDeleteBtn.disabled = true;
        ensureClaim().then(function (held) {
            if (!held) { if (edgeDeleteBtn) edgeDeleteBtn.disabled = false; return; }
            return fetch(API_BASE + '/control/graph/edge/delete', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json', 'X-Claim-Token': claimToken },
                body: JSON.stringify(body),
            }).then(function (resp) {
                if (resp.status === 200) {
                    if (editingEdge) { editingEdge.remove(); editingEdge = null; }
                    closeEdgePanel();
                    persistLayoutSoon();
                    return;
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
                    showEdgeError('Delete failed: ' + (typeof msg === 'string' ? msg : JSON.stringify(msg)));
                });
            });
        }).catch(function (e) {
            showEdgeError('Delete failed: ' + e.message);
        }).then(function () {
            if (edgeDeleteBtn) edgeDeleteBtn.disabled = false;
        });
    }

    if (edgeSaveBtn) edgeSaveBtn.addEventListener('click', saveEdge);
    if (edgeCancelBtn) edgeCancelBtn.addEventListener('click', closeEdgePanel);
    if (edgeDeleteBtn) edgeDeleteBtn.addEventListener('click', deleteEdge);
    window.addEventListener('beforeunload', function () {
        if (claimToken) releaseClaim(true);
    });

    // ── Add node ─────────────────────────────────────────────────────

    var nodeArmEl = document.getElementById('node-arm');
    var nodeRailEl = document.getElementById('node-rail');
    var nodeStatesEl = document.getElementById('node-gripper-states');
    var nodeTransitionsEl = document.getElementById('node-gripper-transitions');
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

    // Populate the form's dropdowns: arm poses from /locations, rail from
    // /track/locations, gripper-state checkboxes from the catalog.
    function populateAddNodeForm(graphData) {
        fetch(API_BASE + '/locations').then(function (r) { return r.json(); })
            .then(function (d) { fillSelect(nodeArmEl, (d && d.locations) || []); })
            .catch(function () {});
        fetch(API_BASE + '/track/locations').then(function (r) { return r.json(); })
            .then(function (d) { fillSelect(nodeRailEl, (d && d.locations) || []); })
            .catch(function () {});
        if (nodeStatesEl) {
            var catalog = (graphData && graphData.gripper_state_catalog) || gripperCatalog;
            nodeStatesEl.innerHTML = Object.keys(catalog).map(function (name) {
                var checked = name === 'empty' ? ' checked' : '';
                return '<label class="checkbox-label"><input type="checkbox" ' +
                    'name="node-gripper-state" value="' + name + '"' + checked + '> ' +
                    leafGlyph(name) + ' ' + name + '</label>';
            }).join('');
        }
    }

    // Parse the optional transitions field: "empty->grip_120, grip_120->empty"
    // into [["empty","grip_120"], ["grip_120","empty"]]. Returns null when
    // empty; throws (with a message) on malformed entries.
    function parseTransitions(raw) {
        if (!raw || !raw.trim()) return null;
        return raw.split(',').map(function (chunk) {
            var pair = chunk.split('->').map(function (s) { return s.trim(); });
            if (pair.length !== 2 || !pair[0] || !pair[1]) {
                throw new Error('Transitions must look like "empty->grip_120" (comma-separated).');
            }
            return pair;
        });
    }

    // Suggest the node id from the pose name when the id box is empty, so
    // the convention (id == the arm pose name) is the default.
    if (nodeArmEl) {
        nodeArmEl.addEventListener('change', function () {
            if (nodeIdEl && !nodeIdEl.value.trim()) nodeIdEl.value = nodeArmEl.value;
        });
    }

    function addNode() {
        var id = (nodeIdEl && nodeIdEl.value.trim()) || '';
        var arm = nodeArmEl && nodeArmEl.value;
        var rail = nodeRailEl && nodeRailEl.value;
        if (!id) { showAddNodeError('Node id is required.'); return; }
        if (!arm || !rail) {
            showAddNodeError('Pose and rail are required.');
            return;
        }
        if (cy && cy.getElementById(id).nonempty()) {
            showAddNodeError('A node with id "' + id + '" already exists.');
            return;
        }
        var tags = (nodeTagsEl && nodeTagsEl.value.trim())
            ? nodeTagsEl.value.split(',').map(function (t) { return t.trim(); }).filter(Boolean)
            : null;
        var states = [];
        if (nodeStatesEl) {
            var checked = nodeStatesEl.querySelectorAll('input[name="node-gripper-state"]:checked');
            Array.prototype.forEach.call(checked, function (cb) { states.push(cb.value); });
        }
        if (!states.length) { showAddNodeError('Pick at least one gripper state.'); return; }
        var transitions;
        try {
            transitions = parseTransitions(nodeTransitionsEl ? nodeTransitionsEl.value : '');
        } catch (e) {
            showAddNodeError(e.message);
            return;
        }
        var body = { id: id, arm: arm, rail: rail, tags: tags, gripper_states: states };
        if (transitions) body.gripper_transitions = transitions;
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
                        if (nodeTransitionsEl) nodeTransitionsEl.value = '';
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
            gripper_states: created.gripper_states || ['empty'],
            gripper_transitions: created.gripper_transitions || [],
            tags: created.tags || [],
        });
        var station = nd.station;
        if (station && !stationAnchors[station]) {
            // First node of a brand-new station becomes its home/anchor.
            stationAnchors[station] = nd.id;
            nd.isAnchor = true;
            nd.count = 1;
        } else if (station) {
            // Bump the home node's member count and reveal the station so the
            // new sibling fans out from the home node.
            var home = cy.getElementById(stationAnchors[station]);
            if (home.nonempty()) home.data('count', (home.data('count') || 1) + 1);
            expanded[station] = true;
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
    var drawCreateBtn = document.getElementById('draw-create-btn');
    var drawCancelBtn = document.getElementById('draw-cancel-btn');
    var drawErrEl = document.getElementById('edge-draw-error');

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

    // Show the create form. Edges never change the gripper — grip/release
    // lives on the nodes' gripper_transitions, so the form is mode/speed only.
    function openDrawForm() {
        clearDrawError();
        if (drawFormEl) drawFormEl.hidden = false;
    }

    function createEdge() {
        if (!drawFrom || !drawTo) { showDrawError('Pick a source and a target node first.'); return; }
        var body = { from_node: drawFrom, to_node: drawTo, mode: drawModeEl ? drawModeEl.value : 'joint' };
        var speedRaw = drawSpeedEl ? drawSpeedEl.value : '';
        if (speedRaw !== '' && speedRaw != null) {
            var sp = parseFloat(speedRaw);
            if (isNaN(sp) || sp <= 0) { showDrawError('Speed must be a number greater than 0.'); return; }
            body.speed = sp;
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
        cy.add({ group: 'edges', data: {
            id: 'e:' + created.from + '->' + created.to,
            source: created.from, target: created.to,
            label: edgeLabel(created.mode, created.speed, created.preconditions || []),
            kind: 'plain', mode: created.mode, speed: created.speed,
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
        var stroke = live.gripper_stroke;
        var state = live.gripper_state || null;
        if (gripperEl) {
            gripperEl.textContent = state
                || (stroke != null ? ('stroke ' + stroke) : '—');
        }

        if (!cy) return;
        // Holding = a named non-empty state; fall back to the stroke
        // heuristic when the state is off-catalog.
        var gripping = node && (state
            ? state !== 'empty'
            : (stroke != null && stroke < 149.5));

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
                if (gripping) ele.addClass('holding');
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
        // Fetch the topology and the device-stored geometry together. The
        // server layout is authoritative across PCs; fall back to the
        // localStorage copy (already in savedLayout) only when the device has
        // nothing saved yet.
        Promise.all([fetchGraph(), fetchLayout()]).then(function (res) {
            var data = res[0], layout = res[1];
            if (layout && layout.positions && Object.keys(layout.positions).length) {
                savedLayout = {
                    positions: layout.positions,
                    expanded: layout.expanded || {},
                    pan: layout.pan || null,
                    zoom: layout.zoom || null,
                };
                expanded = savedLayout.expanded;
            }
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
                    gripper_stroke: mg.gripper_stroke,
                    gripper_state: mg.gripper_state,
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

    // Collapsible side cards: clicking a .card-toggle header shows/hides the
    // sibling .card-body and flips the caret.
    function initCollapsibleCards() {
        var toggles = document.querySelectorAll('.card-toggle');
        Array.prototype.forEach.call(toggles, function (h) {
            h.addEventListener('click', function () {
                var body = h.parentElement.querySelector('.card-body');
                if (!body) return;
                body.hidden = !body.hidden;
                var caret = h.querySelector('.card-caret');
                if (caret) caret.textContent = body.hidden ? '▸' : '▾';
            });
        });
    }

    document.addEventListener('DOMContentLoaded', function () {
        initCollapsibleCards();
        var fitBtn = document.getElementById('fit-btn');
        var resetBtn = document.getElementById('reset-layout-btn');
        if (fitBtn) fitBtn.addEventListener('click', function () { if (cy) cy.fit(undefined, 30); });
        if (resetBtn) resetBtn.addEventListener('click', resetLayout);
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
