/* "Depth Camera" card — the Intel RealSense plugged into this device PC.
 *
 * Reads GET {apiBase}/realsense/status to decide whether to show the card
 * (configured) and what to say when there is no picture (reason). The live
 * preview is a plain <img> pointed at /realsense/stream.mjpg (MJPEG, paced to
 * 10 fps server-side); Color/Depth swap the query string. Clicking the image
 * asks /realsense/depth for the metric distance under the cursor and pins a
 * marker with the reading — the same primitive a future "did the arm really
 * get there" check will use, exposed here so it can be sanity-checked at the
 * bench. Everything is best-effort: a missing camera or driver just explains
 * itself in the overlay and never touches arm control. See
 * core/realsense_camera.py.
 *
 * Usage:  window.setupRealSenseCard({ apiBase: API_BASE });
 */
(function () {
    'use strict';

    window.setupRealSenseCard = function (opts) {
        opts = opts || {};
        var apiBase = opts.apiBase || '';

        var card = document.getElementById('realsense-card');
        var img = document.getElementById('realsense-img');
        var overlay = document.getElementById('realsense-overlay');
        var overlayText = document.getElementById('realsense-overlay-text');
        var statusEl = document.getElementById('realsense-status');
        var toggleBtn = document.getElementById('realsense-toggle');
        var kindBtns = card ? card.querySelectorAll('[data-rs-kind]') : [];
        var marker = document.getElementById('realsense-marker');
        var readout = document.getElementById('realsense-readout');
        if (!card || !img || !overlay || !toggleBtn) return;   // markup missing -> no-op

        var POLL_MS = 5000;
        var kind = 'color';
        var streaming = false;
        var configured = false;
        var attached = null;          // stream URL currently on the <img>
        var busy = false;             // start/stop in flight
        var pollTimer = null;

        function showOverlay(text) {
            if (overlayText) overlayText.textContent = text;
            overlay.hidden = false;
        }
        function hideOverlay() { overlay.hidden = true; }

        function request(path, options) {
            return fetch(apiBase + path, Object.assign({ credentials: 'same-origin' }, options || {}))
                .then(function (r) {
                    return r.json().catch(function () { return {}; }).then(function (body) {
                        if (!r.ok) {
                            var d = body && body.detail;
                            var msg = (d && (d.reason || d.error)) || (typeof d === 'string' ? d : null)
                                || ('HTTP ' + r.status);
                            throw new Error(msg);
                        }
                        return body;
                    });
                });
        }

        // --- Stream attach/detach -----------------------------------------
        function streamUrl() {
            return apiBase + '/realsense/stream.mjpg?stream=' + kind + '&fps=10&t=' + Date.now();
        }
        function attach() {
            if (document.hidden) return;                 // no point decoding in a hidden tab
            var url = streamUrl();
            attached = url;
            img.src = url;
            img.hidden = false;
        }
        function detach() {
            attached = null;
            img.removeAttribute('src');
            img.hidden = true;
            clearMarker();
        }
        img.addEventListener('error', function () {
            // The MJPEG socket dropped (camera stopped, login expired, USB
            // yanked). Fall back to the poll, which re-attaches if it can.
            if (attached) { detach(); showOverlay('Stream ended — reconnecting…'); }
        });
        document.addEventListener('visibilitychange', function () {
            if (document.hidden) { if (attached) detach(); }
            else if (streaming && configured) attach();
        });

        // --- Depth readout on click ---------------------------------------
        function clearMarker() {
            if (marker) marker.hidden = true;
            if (readout) readout.textContent = '';
        }
        img.addEventListener('click', function (ev) {
            if (!streaming || !img.naturalWidth) return;
            // The stage letterboxes with object-fit: contain; map the click
            // back to frame pixels through the rendered image box.
            var rect = img.getBoundingClientRect();
            var scale = Math.min(rect.width / img.naturalWidth, rect.height / img.naturalHeight);
            var drawW = img.naturalWidth * scale, drawH = img.naturalHeight * scale;
            var offX = (rect.width - drawW) / 2, offY = (rect.height - drawH) / 2;
            var px = Math.round((ev.clientX - rect.left - offX) / scale);
            var py = Math.round((ev.clientY - rect.top - offY) / scale);
            if (px < 0 || py < 0 || px >= img.naturalWidth || py >= img.naturalHeight) return;
            if (marker) {
                marker.style.left = (ev.clientX - rect.left) + 'px';
                marker.style.top = (ev.clientY - rect.top) + 'px';
                marker.hidden = false;
            }
            if (readout) readout.textContent = 'measuring…';
            request('/realsense/depth?x=' + px + '&y=' + py + '&window=5')
                .then(function (d) {
                    if (!readout) return;
                    if (d.distance_m == null) { readout.textContent = 'no depth at (' + px + ', ' + py + ')'; return; }
                    var p = d.point_m;
                    readout.textContent = d.distance_m.toFixed(3) + ' m at (' + px + ', ' + py + ')'
                        + (p ? '  ·  xyz ' + p.map(function (v) { return v.toFixed(3); }).join(', ') + ' m' : '');
                })
                .catch(function (e) { if (readout) readout.textContent = 'depth: ' + e.message; });
        });

        // --- Controls -------------------------------------------------------
        Array.prototype.forEach.call(kindBtns, function (btn) {
            btn.addEventListener('click', function () {
                kind = btn.getAttribute('data-rs-kind') === 'depth' ? 'depth' : 'color';
                Array.prototype.forEach.call(kindBtns, function (b) {
                    b.classList.toggle('is-on', b === btn);
                });
                clearMarker();
                if (streaming) attach();
            });
        });

        toggleBtn.addEventListener('click', function () {
            if (busy) return;
            busy = true;
            toggleBtn.disabled = true;
            var path = streaming ? '/realsense/stop' : '/realsense/start';
            showOverlay(streaming ? 'Stopping…' : 'Starting camera…');
            request(path, { method: 'POST' })
                .then(function (d) { render(d); })
                .catch(function (e) { showOverlay(e.message); })
                .then(function () { busy = false; toggleBtn.disabled = false; poll(); });
        });

        // --- Render from /realsense/status -----------------------------------
        function render(d) {
            configured = !!(d && d.configured);
            card.hidden = !configured;
            if (!configured) { if (attached) detach(); return; }

            streaming = !!d.streaming;
            toggleBtn.textContent = streaming ? 'Stop' : 'Start';
            toggleBtn.disabled = busy || (!streaming && !d.installed);
            var dev = d.device || (d.devices && d.devices[0]) || null;
            var bits = [];
            if (dev && dev.name) bits.push(dev.name.replace(/^Intel\(R\) RealSense\(TM\)\s*/, ''));
            if (dev && dev.usb_type) bits.push('USB ' + dev.usb_type);
            if (streaming && d.fps_measured) bits.push(d.fps_measured + ' fps');
            if (streaming && d.streams && d.streams.color) {
                bits.push(d.streams.color.width + '×' + d.streams.color.height);
            }
            if (statusEl) statusEl.textContent = bits.join(' · ');

            if (streaming) {
                if (!attached && !document.hidden) attach();
                if (attached) hideOverlay();
            } else {
                if (attached) detach();
                var why = (d.warnings && d.warnings[0]) || d.reason || 'Camera idle';
                if (!d.installed) why = 'Driver missing: run uv sync --extra realsense';
                else if (d.devices && !d.devices.length) why = 'No RealSense detected — check the USB 3 cable';
                else if (d.state === 'error') why = d.reason || 'Camera error';
                showOverlay(why);
            }
        }

        function poll() {
            clearTimeout(pollTimer);
            request('/realsense/status')
                .then(render)
                .catch(function () { /* API down; leave the card as it was */ })
                .then(function () { pollTimer = setTimeout(poll, POLL_MS); });
        }

        card.hidden = true;
        showOverlay('Connecting…');
        poll();

        return {
            refresh: poll,
            setKind: function (k) { kind = k === 'depth' ? 'depth' : 'color'; if (streaming) attach(); },
        };
    };
})();
