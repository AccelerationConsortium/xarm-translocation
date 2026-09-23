/* "Depth Camera" cards — the Intel RealSense units plugged into this device PC.
 *
 * Reads GET {apiBase}/realsense/cameras first: that is the only endpoint whose
 * path this file knows. It answers with one entry per configured camera, each
 * carrying a ready-made `urls` block, so every other request here follows a
 * URL the server handed over instead of a path this file built — a camera id
 * is device-local configuration, and the panel should not have to agree with
 * the YAML about how it is spelled.
 *
 * One card is cloned from #realsense-card-template per camera and appended to
 * #realsense-cards; nothing renders when no camera is configured. With two or
 * more cameras only one card is shown at a time: each card carries a picker
 * (one button per camera), and the hidden cards drop their stream so an
 * unseen camera costs no USB bandwidth (its pipeline then idles out on the
 * server's idle_timeout_seconds). The choice is remembered per browser. Per
 * card the
 * behaviour is unchanged: the live preview is a plain <img> pointed at the
 * camera's stream.mjpg (MJPEG, paced to 10 fps server-side), Color/Depth swap
 * the query string, and clicking the image asks that camera's /depth for the
 * metric distance under the cursor and pins a marker with the reading — the
 * same primitive a future "did the arm really get there" check will use,
 * exposed here so it can be sanity-checked at the bench. Everything is
 * best-effort: a missing camera or driver just explains itself in the overlay
 * and never touches arm control. See core/realsense_camera.py.
 *
 * Usage:  window.setupRealSenseCard({ apiBase: API_BASE });
 */
(function () {
    'use strict';

    var POLL_MS = 5000;

    window.setupRealSenseCard = function (opts) {
        opts = opts || {};
        var apiBase = opts.apiBase || '';

        var host = document.getElementById('realsense-cards');
        var template = document.getElementById('realsense-card-template');
        if (!host || !template) return;            // markup missing -> no-op

        var cards = {};          // camera id -> card controller
        var order = [];          // camera ids in listing order
        var entries = {};        // camera id -> listing entry
        var activeId = null;     // the one camera whose card is visible
        var listTimer = null;
        var STORE_KEY = 'xarm.realsense.active';

        function loadActive() {
            try { return window.localStorage.getItem(STORE_KEY); } catch (e) { return null; }
        }
        function saveActive(id) {
            try { window.localStorage.setItem(STORE_KEY, id); } catch (e) { /* storage blocked */ }
        }

        function selectCamera(id) {
            if (!cards[id]) return;
            activeId = id;
            saveActive(id);
            order.forEach(function (cid) { cards[cid].setActive(cid === id); });
            renderPickers();
        }

        // Short model name: the live device's, else the "(D435i, ...)" in the
        // configured label (an unplugged camera has no device), else the id.
        function pickerLabel(entry) {
            var facing = entry.mount && entry.mount.facing;
            var name = entry.device && entry.device.name;
            if (!name) {
                var m = /\((D\d+\w*)/i.exec(entry.label || '');
                name = m ? m[1] : entry.id;
            }
            name = String(name).replace(/^Intel\(R\) RealSense\(TM\)\s*/, '').replace(/^RealSense\s*/, '');
            return facing ? name + ' · ' + facing : name;
        }

        function renderPickers() {
            order.forEach(function (cid) {
                var box = cards[cid].pickerEl;
                if (!box) return;
                box.hidden = order.length < 2;
                box.innerHTML = '';
                if (order.length < 2) return;
                order.forEach(function (id) {
                    var b = document.createElement('button');
                    b.type = 'button';
                    b.className = 'lens-btn' + (id === activeId ? ' is-on' : '');
                    b.setAttribute('role', 'radio');
                    b.setAttribute('aria-checked', id === activeId ? 'true' : 'false');
                    b.title = (entries[id] && entries[id].label) || id;
                    b.textContent = pickerLabel(entries[id] || { id: id });
                    b.addEventListener('click', function () { selectCamera(id); });
                    box.appendChild(b);
                });
            });
        }

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

        // --- One camera's card ------------------------------------------
        function makeCard(entry) {
            var root = template.content.firstElementChild.cloneNode(true);
            host.appendChild(root);

            function el(name) { return root.querySelector('[data-rs="' + name + '"]'); }
            var titleEl = el('title');
            var statusEl = el('status');
            var toggleBtn = el('toggle');
            var img = el('img');
            var marker = el('marker');
            var overlay = el('overlay');
            var overlayText = el('overlay-text');
            var readout = el('readout');
            var kindBtns = root.querySelectorAll('[data-rs-kind]');
            var pickerEl = el('cams');

            // Every path for this camera comes from the listing; the fallbacks
            // only matter if an older server answers without `urls`.
            var urls = entry.urls || {};
            var base = '/realsense/' + entry.id;
            function url(key, fallback) { return urls[key] || (base + fallback); }

            var kind = 'color';
            var streaming = false;
            var attached = null;          // stream URL currently on the <img>
            var busy = false;             // start/stop in flight
            var pollTimer = null;
            var disposed = false;
            var active = true;            // false: card hidden, no stream held

            if (titleEl) titleEl.textContent = entry.label || ('Depth Camera · ' + entry.id);
            root.setAttribute('data-camera-id', entry.id);
            root.hidden = false;

            function showOverlay(text) {
                if (overlayText) overlayText.textContent = text;
                overlay.hidden = false;
            }
            function hideOverlay() { overlay.hidden = true; }

            function streamUrl() {
                return apiBase + url('stream', '/stream.mjpg')
                    + '?stream=' + kind + '&fps=10&t=' + Date.now();
            }
            function attach() {
                if (document.hidden || disposed || !active) return;   // no point decoding unseen frames
                var next = streamUrl();
                attached = next;
                img.src = next;
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
                else if (streaming && active) attach();
            });

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
                request(url('depth', '/depth') + '?x=' + px + '&y=' + py + '&window=5')
                    .then(function (d) {
                        if (!readout) return;
                        if (d.distance_m == null) { readout.textContent = 'no depth at (' + px + ', ' + py + ')'; return; }
                        var p = d.point_m;
                        readout.textContent = d.distance_m.toFixed(3) + ' m at (' + px + ', ' + py + ')'
                            + (p ? '  ·  xyz ' + p.map(function (v) { return v.toFixed(3); }).join(', ') + ' m' : '');
                    })
                    .catch(function (e) { if (readout) readout.textContent = 'depth: ' + e.message; });
            });

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
                var path = streaming ? (base + '/stop') : (base + '/start');
                showOverlay(streaming ? 'Stopping…' : 'Starting camera…');
                request(path, { method: 'POST' })
                    .then(function (d) { render(d); })
                    .catch(function (e) { showOverlay(e.message); })
                    .then(function () { busy = false; toggleBtn.disabled = false; poll(); });
            });

            // --- Render from /realsense/<id>/status ----------------------
            function render(d) {
                if (!d) return;
                streaming = !!d.streaming;
                toggleBtn.textContent = streaming ? 'Stop' : 'Start';
                toggleBtn.disabled = busy || (!streaming && (!d.installed || d.present === false));
                // `devices` is the whole bus; only trust it when this camera is on it.
                var dev = d.device || (d.present !== false && d.devices && d.devices[0]) || null;
                var bits = [entry.id];
                if (dev && dev.name) bits.push(dev.name.replace(/^Intel\(R\) RealSense\(TM\)\s*/, ''));
                if (dev && dev.usb_type) bits.push('USB ' + dev.usb_type);
                if (d.mount && d.mount.facing) bits.push('facing ' + d.mount.facing);
                if (streaming && d.fps_measured) bits.push(d.fps_measured + ' fps');
                if (streaming && d.streams && d.streams.color) {
                    bits.push(d.streams.color.width + '×' + d.streams.color.height);
                }
                if (statusEl) statusEl.textContent = bits.join(' · ');

                if (streaming) {
                    if (!attached && !document.hidden && active) attach();
                    if (attached) hideOverlay();
                } else {
                    if (attached) detach();
                    var why = (d.warnings && d.warnings[0]) || d.reason || 'Camera idle';
                    if (!d.installed) why = 'Driver missing: run uv sync --extra realsense';
                    else if (d.devices && !d.devices.length) why = 'No RealSense detected — check the USB 3 cable';
                    else if (d.present === false) why = 'Not connected — ' + (d.reason || 'plug this camera in');
                    else if (d.state === 'error') why = d.reason || 'Camera error';
                    showOverlay(why);
                }
            }

            function poll() {
                clearTimeout(pollTimer);
                if (disposed) return;
                request(url('status', '/status'))
                    .then(render)
                    .catch(function () { /* API down; leave the card as it was */ })
                    .then(function () { if (!disposed) pollTimer = setTimeout(poll, POLL_MS); });
            }

            showOverlay('Connecting…');
            poll();

            return {
                pickerEl: pickerEl,
                setActive: function (on) {
                    active = !!on;
                    root.hidden = !active;
                    if (!active && attached) detach();
                    else if (active && streaming) attach();
                },
                refresh: poll,
                setKind: function (k) { kind = k === 'depth' ? 'depth' : 'color'; if (streaming) attach(); },
                dispose: function () {
                    disposed = true;
                    clearTimeout(pollTimer);
                    if (attached) detach();
                    if (root.parentNode) root.parentNode.removeChild(root);
                },
            };
        }

        // --- The camera list ---------------------------------------------
        // Re-read periodically as well: the registry is built from the YAML at
        // service start, so a restarted service with a new camera shows up in
        // the panel without a reload.
        function syncCameras(body) {
            var listed = (body && body.cameras) || [];
            var seen = {};
            order = [];
            listed.forEach(function (entry) {
                if (!entry || !entry.id) return;
                seen[entry.id] = true;
                order.push(entry.id);
                entries[entry.id] = entry;
                if (!cards[entry.id]) cards[entry.id] = makeCard(entry);
            });
            Object.keys(cards).forEach(function (id) {
                if (!seen[id]) { cards[id].dispose(); delete cards[id]; delete entries[id]; }
            });
            if (!order.length) { activeId = null; return; }
            // Keep the current choice; else the remembered one; else the first.
            var want = (activeId && cards[activeId]) ? activeId : loadActive();
            selectCamera(cards[want] ? want : order[0]);
        }

        function pollList() {
            clearTimeout(listTimer);
            request('/realsense/cameras')
                .then(syncCameras)
                .catch(function () { /* API down; leave the cards as they were */ })
                .then(function () { listTimer = setTimeout(pollList, POLL_MS * 6); });
        }

        pollList();

        return {
            refresh: function () {
                pollList();
                Object.keys(cards).forEach(function (id) { cards[id].refresh(); });
            },
            cards: cards,
        };
    };
})();
