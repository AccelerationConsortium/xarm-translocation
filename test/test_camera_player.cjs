// Run with: node --test test/test_camera_player.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const code = fs.readFileSync(require('node:path').join(__dirname, '../src/web/camera-player.js'), 'utf8');
const settle = () => new Promise(resolve => setImmediate(resolve));
const response = (body, status = 200) => ({ ok: status < 400, status, json: async () => body });

function harness({ hidden = false, sessionResponse, heartbeatResponse } = {}) {
    const calls = [], sockets = [], media = [], timers = new Map(), intervals = [];
    let timerId = 0, sessionId = 0;
    class Events {
        constructor() { this.events = {}; }
        addEventListener(name, cb) { (this.events[name] ||= []).push(cb); }
        emit(name, data) { for (const cb of this.events[name] || []) cb(data); }
    }
    const elements = {};
    for (const id of ['card', 'video', 'overlay', 'overlay-text', 'follow-switch', 'follow-checkbox', 'status']) {
        const e = elements['camera-' + id] = new Events();
        const classes = new Set();
        e.classList = { add: c => classes.add(c), remove: c => classes.delete(c), contains: c => classes.has(c),
            toggle: (c, on) => on ? classes.add(c) : classes.delete(c) };
        e.removeAttribute = name => { delete e[name]; };
        e.load = () => {};
        e.play = () => Promise.resolve();
    }
    const document = Object.assign(new Events(), { hidden, getElementById: id => elements[id] });
    const window = Object.assign(new Events(), { location: new URL('https://lab.example/xarm5/web/') });
    class Socket {
        constructor(url) { this.url = url; this.sent = []; sockets.push(this); }
        send(data) { this.sent.push(JSON.parse(data)); }
        close() { this.closed = true; this.onclose?.(); }
        message(data) { this.onmessage({ data }); }
    }
    class Media extends Events {
        static isTypeSupported() { return true; }
        constructor() { super(); media.push(this); }
        addSourceBuffer() {
            const sb = this.buffer = new Events();
            sb.appended = [];
            sb.appendBuffer = data => { sb.appended.push(data); sb.updating = true; };
            return sb;
        }
    }
    class ObjectURL extends URL {
        static createObjectURL() { return 'blob:video'; }
        static revokeObjectURL() {}
    }
    window.MediaSource = Media;
    const config = { configured: true, available: true, connected: false,
        stream_url: 'ws://old-relay/streams/api/ws?src=cam_tele',
        lenses: [{ id: 'wide', stream_url: 'ws://old-relay/streams/api/ws?src=cam_wide' }] };
    const fetch = async (url, options = {}) => {
        calls.push({ url, ...options });
        if (url.endsWith('/camera/config')) return response(config);
        if (url.endsWith('/heartbeat')) return heartbeatResponse?.() || response({});
        if (options.method === 'DELETE') return response({});
        if (url === '/api/camera-streams/sessions') {
            const id = String(++sessionId);
            return sessionResponse?.(id) || response({ id, ticket: 'ticket-' + id, heartbeat_seconds: 20 });
        }
        throw Error('Unexpected request: ' + url);
    };
    vm.runInNewContext(code, { window, document, fetch, MediaSource: Media, WebSocket: Socket, URL: ObjectURL,
        setTimeout: (cb, ms) => { const id = ++timerId; timers.set(id, { cb, ms }); return id; },
        clearTimeout: id => timers.delete(id), setInterval: cb => intervals.push(cb) });
    const card = window.setupCameraCard({ apiBase: 'https://lab.example/xarm5' });
    return { calls, sockets, media, document, window, card, config, elements, intervals,
        async tick(ms) {
            for (const [id, timer] of [...timers]) if (timer.ms === ms) { timers.delete(id); timer.cb(); }
            await settle();
        } };
}

test('authenticates before MSE, preserves init segment, and shows video only when playing', async () => {
    const h = harness(); await settle();
    assert.equal(h.calls[1].url, '/api/camera-streams/sessions');
    assert.equal(h.calls[1].credentials, 'same-origin');
    assert.deepEqual(JSON.parse(h.calls[1].body), { stream: 'cam_tele' });
    const s = h.sockets[0];
    assert.equal(s.url, 'wss://lab.example/api/camera-streams/ws');
    s.onopen();
    assert.deepEqual(s.sent.map(m => m.type), ['session', 'mse']);
    assert.equal(s.sent[0].value, 'ticket-1');
    s.message(JSON.stringify({ type: 'mse', value: 'video/mp4' }));
    const init = new ArrayBuffer(8), frame = new ArrayBuffer(16);
    s.message(init); s.message(frame);
    h.media[0].emit('sourceopen');
    assert.deepEqual(h.media[0].buffer.appended, [init]);
    h.media[0].buffer.updating = false;
    h.media[0].buffer.emit('updateend');
    assert.deepEqual(h.media[0].buffer.appended, [init, frame]);
    assert.equal(h.elements['camera-overlay'].hidden, false);
    h.elements['camera-video'].emit('playing');
    assert.equal(h.elements['camera-overlay'].hidden, true);
    await h.tick(20000);
    assert.ok(h.calls.some(c => c.url.endsWith('/1/heartbeat') && c.method === 'POST'));
});

test('reconnects with a fresh session and secure URL', async () => {
    const h = harness(); await settle();
    h.sockets[0].onopen(); h.sockets[0].close();
    assert.ok(h.calls.some(c => c.method === 'DELETE' && c.url.endsWith('/1')));
    await h.tick(3000);
    h.sockets[1].onopen();
    assert.equal(h.sockets[1].url, 'wss://lab.example/api/camera-streams/ws');
    assert.equal(h.sockets[1].sent[0].value, 'ticket-2');
    await h.tick(20000);
    assert.ok(!h.calls.some(c => c.url.endsWith('/1/heartbeat')));
});

test('lens changes release the old session and request the selected source', async () => {
    const h = harness(); await settle();
    h.card.setLens('wide'); await settle();
    assert.equal(h.sockets[0].closed, true);
    assert.ok(h.calls.some(c => c.method === 'DELETE' && c.url.endsWith('/1')));
    const minted = h.calls.filter(c => c.url === '/api/camera-streams/sessions');
    assert.equal(JSON.parse(minted[1].body).stream, 'cam_wide');
});

test('does not leak a session minted after navigation', async () => {
    let finish;
    const h = harness({ sessionResponse: () => new Promise(resolve => { finish = resolve; }) });
    await settle(); h.window.emit('pagehide');
    finish(response({ id: 'late', ticket: 'unused', heartbeat_seconds: 20 })); await settle();
    assert.equal(h.sockets.length, 0);
    assert.ok(h.calls.some(c => c.method === 'DELETE' && c.url.endsWith('/late') && c.keepalive));
});

test('hidden tabs pause, release capacity, and resume on return', async () => {
    const h = harness({ hidden: true }); await settle();
    assert.equal(h.sockets.length, 0);
    h.document.hidden = false; h.document.emit('visibilitychange'); await settle();
    assert.equal(h.sockets.length, 1);
    h.document.hidden = true; h.document.emit('visibilitychange'); await settle();
    assert.equal(h.sockets[0].closed, true);
    await h.tick(3000);
    assert.equal(h.sockets.length, 1);
});

for (const [status, detail, message] of [
    [401, 'Unauthorized', 'Sign in to view this camera'],
    [403, 'Your account has no access to this camera', 'Your account has no access to this camera'],
    [429, 'Camera capacity is in use.', 'Camera capacity is in use.'],
    [404, 'Not Found', 'Open the control interface through the lab dashboard to view this camera'],
]) test('surfaces session error ' + status + ' without falling back to the retired relay', async () => {
    const h = harness({ sessionResponse: () => response({ detail }, status) }); await settle();
    assert.equal(h.sockets.length, 0);
    assert.equal(h.elements['camera-overlay-text'].textContent, message);
});

test('revoked heartbeat stops video and releases the session', async () => {
    const h = harness({ heartbeatResponse: () => response({ detail: 'Permission expired' }, 403) }); await settle();
    h.sockets[0].onopen(); await h.tick(20000);
    assert.equal(h.sockets[0].closed, true);
    assert.equal(h.elements['camera-overlay-text'].textContent, 'Permission expired');
    assert.ok(h.calls.some(c => c.method === 'DELETE'));
});

test('broker errors stop video and display the reason', async () => {
    const h = harness(); await settle();
    h.sockets[0].message(JSON.stringify({ type: 'session/error', value: 'Viewing session expired' }));
    assert.equal(h.sockets[0].closed, true);
    assert.equal(h.elements['camera-overlay-text'].textContent, 'Viewing session expired');
});
