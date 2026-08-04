/* Shared "Lab Camera" card — used by both the control panel (index.html)
 * and the motion-graph page (graph.html). Both pages carry the same card
 * markup (ids: camera-card, camera-video, camera-overlay, camera-overlay-text,
 * camera-follow-switch, camera-follow-checkbox, camera-status).
 *
 * Reads GET {apiBase}/camera/config to decide whether to show the card
 * (configured) and whether the live preview is usable right now (available +
 * stream_url). The preview is a go2rtc MSE stream over WebSocket; "Follow arm"
 * toggles POST {apiBase}/camera/follow. Everything here is best-effort — a
 * camera outage or missing config just hides/greys the card and never touches
 * arm control. See core/camera_tracker.py.
 *
 * Usage:  window.setupCameraCard({ apiBase: API_BASE });
 */
(function () {
    'use strict';

    window.setupCameraCard = function (opts) {
        opts = opts || {};
        var apiBase = opts.apiBase || '';
        var selectedLens = null;      // set by the panel's Wide/Tele buttons
        var lastLenses = [];

        var card = document.getElementById('camera-card');
        var video = document.getElementById('camera-video');
        var overlay = document.getElementById('camera-overlay');
        var overlayText = document.getElementById('camera-overlay-text');
        var sw = document.getElementById('camera-follow-switch');
        var checkbox = document.getElementById('camera-follow-checkbox');
        var statusEl = document.getElementById('camera-status');
        if (!card || !video || !sw || !checkbox) return;   // markup missing -> no-op

        var POLL_MS = 10000;
        var RECONNECT_MS = 3000;
        var mseSupported = typeof window.MediaSource !== 'undefined';

        var configured = false;
        var connected = false;
        var streamUrl = null;        // stream currently attached to the player
        var ws = null;               // live MSE websocket
        var sourceBuffer = null;
        var pendingBuffers = [];     // segments waiting on the SourceBuffer
        var reconnectTimer = null;
        var toggling = false;        // suppress poll-driven state churn mid-toggle

        function showOverlay(text) {
            overlayText.textContent = text;
            overlay.hidden = false;
        }
        function hideOverlay() { overlay.hidden = true; }

        // --- Keep playback at the live edge -------------------------------
        // A live MSE stream plays at 1x from wherever decoding began, so any
        // buffer accumulated at start (or after a stall / backgrounded tab)
        // becomes permanent latency. Nudge toward live: ease the rate up on a
        // small lead, hard-seek if we fall badly behind. Bounds the delay to
        // ~LIVE_TARGET instead of letting it creep. (LAB camera has no audio,
        // so a slightly faster rate is imperceptible.)
        var LIVE_TARGET = 0.35;   // aim to sit this far behind the live edge (s)
        var LIVE_NUDGE = 0.9;     // ease toward live once lead exceeds this (s)
        var LIVE_RESYNC = 3.0;    // hard-seek to live once lead exceeds this (s)
        function keepLiveEdge() {
            if (!sourceBuffer) return;
            var b;
            try { b = video.buffered; } catch (e) { return; }
            if (!b || !b.length) return;
            var end = b.end(b.length - 1);
            var lead = end - video.currentTime;
            if (lead > LIVE_RESYNC) {
                try { video.currentTime = end - LIVE_TARGET; } catch (e) {}
                video.playbackRate = 1.0;
            } else if (lead > LIVE_NUDGE) {
                video.playbackRate = 1.08;   // smooth catch-up, no visible jump
            } else if (video.playbackRate !== 1.0) {
                video.playbackRate = 1.0;
            }
        }
        video.addEventListener('timeupdate', keepLiveEdge);

        // --- HTTP helpers (plain fetch; follow is login-gated by cookie, not
        //     claim-gated, so no token plumbing is needed). ---
        function getConfig() {
            return fetch(apiBase + '/camera/config', { credentials: 'same-origin' })
                .then(function (r) { return r.ok ? r.json() : null; })
                .catch(function () { return null; });
        }
        function postFollow(enabled) {
            return fetch(apiBase + '/camera/follow', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                credentials: 'same-origin',
                body: JSON.stringify({ enabled: enabled }),
            })
                .then(function (r) { return r.ok ? r.json() : null; })
                .catch(function () { return null; });
        }

        // --- MSE player (go2rtc) ---
        function stopStream() {
            if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
            if (ws) {
                try { ws.onclose = null; ws.onerror = null; ws.onmessage = null; ws.close(); } catch (e) {}
                ws = null;
            }
            sourceBuffer = null;
            pendingBuffers = [];
            try {
                video.playbackRate = 1.0;
                if (video.src) { URL.revokeObjectURL(video.src); }
                video.removeAttribute('src');
                video.load();
            } catch (e) {}
        }

        function scheduleReconnect() {
            if (ws) { try { ws.onclose = null; ws.close(); } catch (e) {} ws = null; }
            sourceBuffer = null;
            pendingBuffers = [];
            if (!configured || !streamUrl || reconnectTimer) return;
            reconnectTimer = setTimeout(function () {
                reconnectTimer = null;
                if (configured && streamUrl) connectMse(streamUrl);
            }, RECONNECT_MS);
        }

        // Append a segment, trimming the buffer if the browser is out of room
        // (a live stream would otherwise grow without bound).
        function appendSegment(buf) {
            if (!sourceBuffer) return;
            try {
                sourceBuffer.appendBuffer(buf);
            } catch (e) {
                if (e && e.name === 'QuotaExceededError' && sourceBuffer.buffered.length) {
                    var trimTo = Math.max(sourceBuffer.buffered.start(0), video.currentTime - 5);
                    if (trimTo > sourceBuffer.buffered.start(0)) {
                        pendingBuffers.unshift(buf);
                        try { sourceBuffer.remove(sourceBuffer.buffered.start(0), trimTo); } catch (e2) {}
                    }
                }
                // Any other append error: let the socket's onclose reconnect.
            }
        }

        function connectMse(url) {
            var socket;
            try { socket = new WebSocket(url); }
            catch (e) { scheduleReconnect(); return; }
            socket.binaryType = 'arraybuffer';
            ws = socket;
            var gotData = false;   // has a real video segment arrived yet?

            socket.onopen = function () {
                // Ask go2rtc for MSE using only the codecs this browser can play.
                // Codec preference mirrors the dashboard's go2rtc.ts — note the
                // H.264 main/baseline entries most Tapo C-series cameras emit.
                var candidates = [
                    'avc1.640029', 'avc1.64002A', 'avc1.640033',
                    'avc1.4D401E', 'avc1.42E01E',
                    'hvc1.1.6.L153.B0',
                    'mp4a.40.2', 'mp4a.40.5', 'flac', 'opus'
                ];
                var supported = candidates.filter(function (c) {
                    return MediaSource.isTypeSupported('video/mp4; codecs="' + c + '"');
                });
                try { socket.send(JSON.stringify({ type: 'mse', value: supported.join(',') })); } catch (e) {}
            };

            // Drain queued segments one at a time (appendBuffer is async, so we
            // append again on each 'updateend'). Critically, we queue frames
            // BEFORE the SourceBuffer exists too: go2rtc sends the init segment
            // (ftyp+moov) as the very first binary frame — before our async
            // 'sourceopen' fires — and dropping it means nothing ever decodes.
            function flush() {
                if (!sourceBuffer || sourceBuffer.updating || !pendingBuffers.length) return;
                appendSegment(pendingBuffers.shift());
                if (!gotData) { gotData = true; hideOverlay(); }   // first frame shown
            }

            socket.onmessage = function (ev) {
                if (ws !== socket) return;   // stale socket
                if (typeof ev.data === 'string') {
                    var msg;
                    try { msg = JSON.parse(ev.data); } catch (e) { return; }
                    if (msg.type === 'mse' || msg.type === 'mp4') startMediaSource(socket, msg.value, flush);
                    return;
                }
                pendingBuffers.push(ev.data);   // never drop — esp. the init segment
                flush();
            };

            socket.onerror = function () { try { socket.close(); } catch (e) {} };
            socket.onclose = function () {
                if (ws !== socket) return;
                // Closed before any video arrived -> tell the operator, then retry.
                if (!gotData) showOverlay('Stream unreachable — retrying…');
                scheduleReconnect();
            };
        }

        function startMediaSource(socket, mime, flush) {
            var media = new MediaSource();
            try { video.src = URL.createObjectURL(media); } catch (e) { scheduleReconnect(); return; }
            media.addEventListener('sourceopen', function () {
                if (ws !== socket) return;   // superseded before it opened
                try { URL.revokeObjectURL(video.src); } catch (e) {}
                var sb;
                try {
                    sb = media.addSourceBuffer(mime);
                } catch (e) { scheduleReconnect(); return; }
                sb.mode = 'segments';
                sb.addEventListener('updateend', flush);
                sourceBuffer = sb;
                video.play().catch(function () {});   // autoplay needs muted (it is)
                flush();   // drain segments (incl. the init) queued before now
                // Overlay stays until the first segment actually arrives
                // (see onmessage) so a black frame never reads as "connected".
            }, { once: true });
        }

        function startStream(url) {
            if (url === streamUrl && ws) return;   // already streaming this url
            stopStream();
            streamUrl = url;
            if (!mseSupported) {
                showOverlay('Live preview not supported in this browser');
                return;   // camera still pans server-side; only the preview is gone
            }
            // Mixed content: a ws:// stream on an https page is silently blocked
            // by the browser. Behind the single Caddy edge every service shares
            // one origin, so re-anchor the stream path on the page origin as
            // wss:// (same-origin, valid cert). If that host doesn't serve
            // /streams the onclose handler surfaces "unreachable" and retries.
            if (/^ws:\/\//i.test(url) && window.location.protocol === 'https:') {
                try {
                    var u = new URL(url);
                    url = 'wss://' + window.location.host + u.pathname + u.search;
                } catch (e) {
                    showOverlay('Live preview blocked (mixed content: https page, '
                        + 'ws:// stream). Set stream_base_url to a wss:// origin.');
                    return;
                }
            }
            showOverlay('Connecting…');
            connectMse(url);
        }

        // --- Follow toggle ---
        function applyFollowing(on) {
            checkbox.checked = !!on;
            sw.classList.toggle('is-on', !!on);
        }
        function setToggleEnabled(enabled) {
            checkbox.disabled = !enabled;
            sw.classList.toggle('is-disabled', !enabled);
        }

        sw.addEventListener('click', function (e) {
            // It's a <label>; own the toggle so the checkbox tracks the server.
            e.preventDefault();
            if (sw.classList.contains('is-disabled') || toggling) return;
            var next = !checkbox.checked;
            toggling = true;
            postFollow(next).then(function (data) {
                toggling = false;
                if (data) {
                    applyFollowing(!!data.following);
                } else {
                    refresh();   // refused (not connected / locked): re-sync
                }
            });
        });

        // --- Poll /camera/config ---
        function refresh() {
            return getConfig().then(function (data) {
                if (!data || !data.configured) {
                    configured = false;
                    card.hidden = true;
                    card.classList.remove('camera-live');
                    stopStream();
                    return;
                }
                configured = true;
                connected = !!data.connected;
                card.hidden = false;

                // Follow toggle is only actionable with a connected controller
                // (POST /camera/follow needs one). Reflect the reported state.
                if (!toggling) applyFollowing(!!data.following);
                // Hand the camera's saved views to whoever renders them
                // (the arm panel's preset buttons); graph.html has no
                // preset UI, so the hook is optional.
                if (typeof opts.onPresets === 'function') opts.onPresets(data.presets);
                lastLenses = Array.isArray(data.lenses) ? data.lenses : [];
                if (typeof opts.onLenses === 'function') {
                    opts.onLenses(lastLenses, selectedLens);
                }
                setToggleEnabled(connected);
                statusEl.textContent = '';

                var url = data.stream_url;
                if (selectedLens) {
                    for (var i = 0; i < lastLenses.length; i++) {
                        if (lastLenses[i].id === selectedLens && lastLenses[i].stream_url) {
                            url = lastLenses[i].stream_url;
                            break;
                        }
                    }
                }
                if (data.available && url) {
                    card.classList.add('camera-live');   // glowing title dot
                    startStream(url);
                } else {
                    card.classList.remove('camera-live');
                    stopStream();
                    streamUrl = null;
                    showOverlay(data.reason || 'Camera unavailable');
                }
            });
        }

        refresh();
        setInterval(refresh, POLL_MS);

        // Handle for the host page: switching lens restreams immediately
        // rather than waiting for the next poll.
        return {
            setLens: function (id) {
                if (selectedLens === id) return;
                selectedLens = id;
                stopStream();
                streamUrl = null;
                refresh();
            },
            getLens: function () { return selectedLens; },
        };
    };
})();
