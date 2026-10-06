/* Page composition only. Robot actions and the single claim live in main.js;
 * graph.js owns the editor, and camera modules own their existing players. */
(function () {
    'use strict';
    document.addEventListener('DOMContentLoaded', function () {
        function tabs(entries, onChange) {
            function select(key, focus) {
                entries.forEach(function (entry) {
                    var active = entry.key === key;
                    entry.button.classList.toggle('active', active);
                    entry.button.setAttribute('aria-selected', String(active));
                    entry.button.tabIndex = active ? 0 : -1;
                    entry.pane.hidden = !active;
                    if (active && focus) entry.button.focus();
                });
                onChange(key);
            }
            entries.forEach(function (entry, index) {
                entry.button.addEventListener('click', function () { select(entry.key, false); });
                entry.button.addEventListener('keydown', function (event) {
                    var next;
                    if (event.key === 'ArrowRight') next = (index + 1) % entries.length;
                    if (event.key === 'ArrowLeft') next = (index + entries.length - 1) % entries.length;
                    if (event.key === 'Home') next = 0;
                    if (event.key === 'End') next = entries.length - 1;
                    if (next === undefined) return;
                    event.preventDefault();
                    select(entries[next].key, true);
                });
            });
            return select;
        }
        var addNodeToggle = document.getElementById('graph-add-node-toggle');
        var addNodePanel = document.getElementById('add-node-panel');
        function showAddNode(open) {
            addNodePanel.hidden = !open;
            addNodeToggle.classList.toggle('active', open);
            addNodeToggle.setAttribute('aria-expanded', String(open));
            addNodeToggle.setAttribute('aria-pressed', String(open));
            if (open) document.getElementById('node-arm').focus();
        }
        addNodeToggle.addEventListener('click', function () { showAddNode(addNodePanel.hidden); });
        addNodePanel.addEventListener('keydown', function (event) {
            if (event.key === 'Escape') {
                showAddNode(false);
                addNodeToggle.focus();
            }
        });
        var selectGraphDrive = tabs(['reachable', 'travel'].map(function (key) {
            return { key: key, button: document.getElementById('graph-drive-tab-' + key), pane: document.getElementById('graph-drive-pane-' + key) };
        }), function () {});
        selectGraphDrive('reachable', false);
        var routes = { graph: 'graph-drive', arm: 'direct-drive', edit: 'edit-graph' };
        var selectMode = tabs(['graph', 'arm', 'edit'].map(function (key) {
            return { key: key, button: document.getElementById('mode-tab-' + key), pane: document.getElementById('mode-pane-' + key) };
        }), function (key) {
            document.getElementById('drive-layout').hidden = key === 'edit';
            history.replaceState(null, '', location.pathname + location.search + '#' + routes[key]);
            if (key === 'edit') requestAnimationFrame(function () {
                if (window.__graphViewer) window.__graphViewer.resize();
            });
        });
        function readMode() {
            var key = Object.keys(routes).find(function (k) { return '#' + routes[k] === location.hash; }) || 'graph';
            selectMode(key, false);
        }
        readMode();
        window.addEventListener('hashchange', readMode);
        var selectCamera = tabs(['stereo', 'lab'].map(function (key) {
            return { key: key, button: document.getElementById('camera-tab-' + key), pane: document.getElementById('camera-pane-' + key) };
        }), function (key) {
            document.dispatchEvent(new CustomEvent('xarm:camera-view', { detail: key }));
            var url = new URL(location.href);
            if (key === 'lab') url.searchParams.delete('camera');
            else url.searchParams.set('camera', key);
            history.replaceState(null, '', url.pathname + url.search + url.hash);
        });
        selectCamera(new URL(location.href).searchParams.get('camera') === 'stereo' ? 'stereo' : 'lab', false);
    });
})();
