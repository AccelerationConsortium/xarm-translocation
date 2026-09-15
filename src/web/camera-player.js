/* Shared "Lab Camera" card — used by both the control panel (index.html)
 * and the motion-graph page (graph.html). Both pages carry the same card
 * markup (ids: camera-card, camera-video, camera-overlay, camera-overlay-text,
 * camera-follow-switch, camera-follow-checkbox, camera-status).
 *
 * Reads GET {apiBase}/camera/config to decide whether to show the card
 * (configured) and whether the live preview is usable right now (available +
 * stream_url). The preview uses an authenticated dashboard viewing session
 * carrying go2rtc MSE over WebSocket; "Follow arm"
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
        var generation = 0;          // invalidates in-flight session requests
        var connecting = false;
        var releaseSession = null;
        var pageHidden = false;
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
            generation++;
            connecting = false;
            if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
            if (releaseSession) { releaseSession(); releaseSession = null; }
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
            stopStream();
            if (!configured || !streamUrl || document.hidden || pageHidden) return;
            reconnectTimer = setTimeout(function () {
                reconnectTimer = null;
                if (configured && streamUrl) startStream(streamUrl);
            }, RECONNECT_MS);
        }

        // The dashboard closed /streams/* (410). Its replacement is on the
        // shared origin, outside the device's /xarm5 prefix. stream_url only
        // supplies the registered source name; never connect to the raw relay.
        function sessionRequest(path, options) {
            return fetch('/api/camera-streams' + path, Object.assign({
                credentials: 'same-origin', cache: 'no-store',
            }, options)).then(function (response) {
                return response.json().catch(function () { return {}; }).then(function (body) {
                    if (!response.ok) {
                        var message = typeof body.detail === 'string' ? body.detail : 'Camera access unavailable';
                        if (response.status === 401) message = 'Sign in to view this camera';
                        if (response.status === 404) message = 'Open the control interface through the lab dashboard to view this camera';
                        throw new Error(message);
                    }
                    return body;
                });
            });
        }

        function openSession(url) {
            var attempt = generation;
            var source;
            try { source = new URL(url, window.location.href).searchParams.get('src'); }
            catch (e) {}
            if (!source) { showOverlay('Unknown camera feed'); return; }
            connecting = true;
            sessionRequest('/sessions', {
                method: 'POST', headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ stream: source }),
            }).then(function (session) {
                var endpoint = '/sessions/' + encodeURIComponent(session.id);
                var heartbeatTimer = null;
                var ended = false;
                function release() {
                    if (ended) return;
                    ended = true;
                    clearTimeout(heartbeatTimer);
                    // Collect and release even if a lens change or navigation
                    // happened while the POST was in flight.
                    fetch('/api/camera-streams' + endpoint, {
                        method: 'DELETE', credentials: 'same-origin', keepalive: true,
                    }).catch(function () {});
                }
                if (attempt !== generation || document.hidden || pageHidden) { release(); return; }
                releaseSession = release;
                function fail(message) {
                    if (ended || attempt !== generation) return;
                    showOverlay(message);
                    scheduleReconnect();
                }
                function heartbeat() {
                    if (ended) return;
                    sessionRequest(endpoint + '/heartbeat', { method: 'POST' }).then(function () {
                        if (!ended) heartbeatTimer = setTimeout(heartbeat, session.heartbeat_seconds * 1000);
                    }).catch(function (error) { fail(error.message); });
                }
                var socketUrl = new URL('/api/camera-streams/ws', window.location.href);
                socketUrl.protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                connectMse(socketUrl.href, function (socket) {
                    socket.send(JSON.stringify({ type: 'session', value: session.ticket }));
                    session.ticket = '';
                    heartbeatTimer = setTimeout(heartbeat, session.heartbeat_seconds * 1000);
                }, fail);
            }).catch(function (error) {
                if (attempt !== generation) return;
                connecting = false;
                // Config polling retries after sign-in or a capacity change.
                showOverlay(error.message || 'Camera viewing service unreachable');
            });
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

        function connectMse(url, authenticate, fail) {
            var socket;
            try { socket = new WebSocket(url); }
            catch (e) { scheduleReconnect(); return; }
            socket.binaryType = 'arraybuffer';
            ws = socket;
            connecting = false;

            socket.onopen = function () {
                if (ws !== socket) return;
                // The single-use ticket must precede the MSE codec request.
                try { authenticate(socket); } catch (e) { fail('Camera session failed'); return; }
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
            }

            socket.onmessage = function (ev) {
                if (ws !== socket) return;   // stale socket
                if (typeof ev.data === 'string') {
                    var msg;
                    try { msg = JSON.parse(ev.data); } catch (e) { return; }
                    if (msg.type === 'session/error' || msg.type === 'error') {
                        fail(msg.value || 'Camera stream unavailable');
                        return;
                    }
                    if (msg.type === 'mse' || msg.type === 'mp4') startMediaSource(socket, msg.value, flush);
                    return;
                }
                pendingBuffers.push(ev.data);   // never drop — esp. the init segment
                flush();
            };

            socket.onerror = function () { try { socket.close(); } catch (e) {} };
            socket.onclose = function () {
                if (ws !== socket) return;
                // A frozen last frame must also show the outage.
                showOverlay('Stream unreachable — retrying…');
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
                sb.addEventListener('error', function () {
                    if (ws !== socket) return;
                    showOverlay('Video playback failed — retrying…');
                    scheduleReconnect();
                });
                sourceBuffer = sb;
                video.play().catch(function () {});   // autoplay needs muted (it is)
                flush();   // drain segments (incl. the init) queued before now
                // The playing event hides the overlay after decoding starts.
            }, { once: true });
        }

        function startStream(url) {
            if (url === streamUrl && (ws || connecting || reconnectTimer)) return;
            stopStream();
            streamUrl = url;
            if (document.hidden || pageHidden) {
                showOverlay('Video paused while this tab is hidden');
                return;
            }
            if (!mseSupported) {
                showOverlay('Live preview not supported in this browser');
                return;   // camera still pans server-side; only the preview is gone
            }
            showOverlay('Connecting…');
            openSession(url);
        }

        video.addEventListener('playing', function () { if (ws) hideOverlay(); });
        video.addEventListener('error', function () {
            if (!ws) return;
            showOverlay('Video playback failed — retrying…');
            scheduleReconnect();
        });
        document.addEventListener('visibilitychange', function () {
            if (document.hidden) {
                stopStream();
                showOverlay('Video paused while this tab is hidden');
            } else refresh();
        });
        window.addEventListener('pagehide', function () { pageHidden = true; stopStream(); });
        window.addEventListener('pageshow', function () { pageHidden = false; refresh(); });

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
