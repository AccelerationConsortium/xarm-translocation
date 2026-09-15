"""Offline tests: no real sockets or robot-control constructors."""

import io
import math
import sys
from contextlib import contextmanager
from importlib.resources import files

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.ur import URObserver, interpret
from robot_motion.graph import Graph


def graph(model="ur5e", count=6):
    return {
        "robot_model": model,
        "nodes": [
            {"id": "a", "joints_deg": [0] * count},
            {"id": "b", "joints_deg": [1] * count},
            {"id": "c", "joints_deg": [2] * count},
        ],
        "edges": [{"source": "a", "target": "b"}, {"source": "b", "target": "c"}],
    }


def test_import_does_not_load_vendor_modules():
    assert "rtde_control" not in sys.modules
    assert "rtde_receive" not in sys.modules
    assert "xarm" not in sys.modules


@pytest.mark.parametrize(
    "config",
    [
        {"control_enabled": True},
        {"driver": "ur", "model": "mg400"},
        {"driver": "ur", "model": "ur5e", "observe": True},
        {"driver": "none", "observe": True},
        {
            "driver": "mg400",
            "model": "mg400",
            "observe": True,
            "robot_host": "example.invalid",
        },
        {"timeout_s": float("inf")},
    ],
)
def test_invalid_or_control_configuration_is_rejected(config):
    with pytest.raises(ValidationError):
        Settings(**config)


@pytest.mark.parametrize(
    "model,count",
    [("xarm5", 5), ("ur3e", 6), ("ur5e", 6), ("ur5_cb3", 6), ("mg400", 4)],
)
def test_model_specific_graph_and_existing_path_planner(model, count):
    g = Graph.model_validate(graph(model, count))
    assert g.path("a", "c") == ["b", "c"]
    assert g.path("a", "a") == []
    bad = graph(model, count + 1)
    with pytest.raises(ValidationError):
        Graph.model_validate(bad)


def test_graph_rejects_invalid_structure_and_nonfinite_coordinates():
    cases = []
    bad = graph()
    bad["nodes"][0]["joints_deg"][0] = math.nan
    cases.append(bad)
    bad = graph()
    bad["edges"][0]["target"] = "missing"
    cases.append(bad)
    bad = graph()
    bad["nodes"][1]["id"] = "a"
    cases.append(bad)
    bad = graph()
    bad["edges"].append(bad["edges"][0])
    cases.append(bad)
    bad = graph()
    bad["edges"][0]["mode"] = "linear"
    cases.append(bad)
    bad = graph()
    bad["nodes"][0]["id"] = "<script>"
    cases.append(bad)
    for bad in cases:
        with pytest.raises(ValidationError):
            Graph.model_validate(bad)


def test_all_default_ui_and_documentation_assets_are_packaged():
    with TestClient(create_app()) as client:
        for path in [
            "/",
            "/health",
            "/status",
            "/drivers",
            "/graph",
            "/docs",
            "/openapi.json",
            "/agent-docs",
            "/agent-docs/api-reference",
            "/llms.txt",
            "/web/",
            "/web/style.css",
            "/web/main.js",
            "/web/pyxarm/style.css",
            "/web/pyxarm/cytoscape.min.js",
        ]:
            response = client.get(path)
            assert response.status_code == 200, path
        assert "No physical control" in client.get("/web/").text
        status = client.get("/status").json()
        assert status["equipment_status"] == "unknown"
        assert status["allowed_actions"] == []
        assert status["details"]["control_enabled"] is False
        for path in [
            "/control/claim",
            "/control/graph/move_to",
            "/control/startup",
            "/connect",
        ]:
            assert client.post(path, json={}).status_code == 404


def test_shared_assets_are_exact_and_do_not_expose_legacy_controls():
    with TestClient(create_app()) as client:
        for asset in ["style.css", "cytoscape.min.js"]:
            response = client.get("/web/pyxarm/" + asset)
            assert response.content == files("web").joinpath(asset).read_bytes()
        for asset in ["main.js", "graph.js", "index.html", "server.py", "camera-player.js"]:
            assert client.get("/web/pyxarm/" + asset).status_code == 404
        for path in ["/web/pyxarm/%2e%2e/server.py", "/web/pyxarm/%2e%2e%2fserver.py"]:
            assert client.get(path).status_code == 404
        html = client.get("/web/").text
        assert 'src="pyxarm/cytoscape.min.js"' in html
        assert 'href="pyxarm/style.css"' in html
        assert "<iframe" not in html and "<video" not in html
        assert "Direct Drive" in html and "Local draft only" in html
        assert "STOP · unavailable" in html
        assert "UR5e Control Interface" in html
        assert 'id="control-modes-card"' in html
        assert 'class="log-container rm-observer"' in html
        assert "xArm Control" not in html


def test_html_matches_dashboard_asset_and_script_policy():
    from html.parser import HTMLParser

    class Assets(HTMLParser):
        def handle_starttag(self, tag, attrs):
            attributes = dict(attrs)
            assert not any(name.startswith("on") for name in attributes)
            if tag == "script":
                assert attributes.get("src") in {"main.js", "pyxarm/cytoscape.min.js"}
            if tag == "link" and attributes.get("rel") == "stylesheet":
                assert attributes.get("href") in {"style.css", "pyxarm/style.css"}

    Assets().feed(files("robot_motion").joinpath("web/index.html").read_text(encoding="utf-8"))


def test_offline_edits_never_replace_configured_graph_or_status(tmp_path):
    import json

    path = tmp_path / "graph.local.json"
    original = graph()
    path.write_text(json.dumps(original))
    app = create_app(Settings(driver="ur", model="ur5e", graph_file=str(path)))
    with TestClient(app) as client:
        edited = graph()
        edited["nodes"][0]["joints_deg"] = [25] * 6
        assert client.post("/graph/validate", json=edited).status_code == 200
        response = client.post("/graph/preview", json={"graph": edited, "source": "a", "target": "c"})
        assert response.json()["executed"] is False
        assert client.get("/graph").json()["graph"]["nodes"][0]["joints_deg"] == [0] * 6
        status = client.get("/status").json()
        assert status["allowed_actions"] == []
        assert status["details"]["control_enabled"] is False
        assert status["details"]["monitoring_only"] is True
        assert status["equipment_status"] == "unknown"
        assert json.loads(path.read_text()) == original


def test_linear_target_requires_explicit_tcp_and_missing_joints_are_not_zero():
    with TestClient(create_app()) as client:
        candidate = graph()
        candidate["edges"][0]["mode"] = "linear"
        assert client.post("/graph/validate", json=candidate).status_code == 422
        candidate["nodes"][1]["tcp_mm_rpy_deg"] = [100, 200, 300, 0, 0, 0]
        assert client.post("/graph/validate", json=candidate).status_code == 200
        del candidate["nodes"][0]["joints_deg"]
        assert client.post("/graph/validate", json=candidate).status_code == 422


def test_preview_is_offline_and_never_authorizes_hardware():
    with TestClient(create_app()) as client:
        result = client.post(
            "/graph/preview", json={"graph": graph(), "source": "a", "target": "c"}
        )
        assert result.status_code == 200
        assert result.json()["path"] == ["a", "b", "c"]
        assert result.json()["executed"] is False
        assert result.json()["physical_validation"] is False
        assert (
            client.post(
                "/graph/preview", json={"graph": graph(), "source": "c", "target": "a"}
            ).status_code
            == 422
        )
        assert (
            client.post(
                "/graph/preview",
                json={"graph": graph(), "source": "missing", "target": "a"},
            ).status_code
            == 422
        )


class FakeSocket:
    def __init__(self, replies):
        self.data = io.BytesIO(replies)
        self.sent = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def makefile(self, _):
        return self.data

    def sendall(self, data):
        self.sent.append(data)


@pytest.mark.parametrize(
    "model,safety",
    [
        ("ur5e", b"Safetystatus: NORMAL\n"),
        ("ur5_cb3", b"Command not found\nSafetymode: NORMAL\n"),
    ],
)
def test_readonly_queries_and_cb3_fallback(model, safety):
    sock = FakeSocket(
        b"Connected: Universal Robots Dashboard Server\nRobotmode: RUNNING\n"
        + safety
        + b"PLAYING private-program.urp\n"
    )
    settings = Settings(driver="ur", model=model, robot_host="example.invalid")
    observer = URObserver(settings, connector=lambda *_a, **_k: sock)
    observation = observer.read()
    assert observation["equipment_status"] == "busy"
    assert observation["activity"] == "running"
    assert observation["details"]["program_state"] == "PLAYING"
    assert b"private-program" not in str(observation).encode()
    assert set(sock.sent) <= {
        b"robotmode\n",
        b"safetystatus\n",
        b"safetymode\n",
        b"programState\n",
    }
    assert sock.data.closed


@pytest.mark.parametrize(
    "mode,safety,program,state,activity",
    [
        ("RUNNING", "NORMAL", "PLAYING", "busy", "running"),
        ("RUNNING", "NORMAL", "STOPPED", "ready", "idle"),
        ("RUNNING", "REDUCED", "PLAYING", "degraded", "running"),
        ("RUNNING", "NORMAL", "PAUSED", "degraded", "idle"),
        ("RUNNING", "ROBOT_EMERGENCY_STOP", "STOPPED", "e_stop", "idle"),
        ("RUNNING", "PROTECTIVE_STOP", "STOPPED", "error", "idle"),
        ("RUNNING", "PROTECTIVE_STOP", "PLAYING", "error", "running"),
        ("POWER_OFF", "NORMAL", "STOPPED", "requires_init", "idle"),
        ("RUNNING", "UNRECOGNIZED", "STOPPED", "unknown", "unknown"),
    ],
)
def test_status_contract(mode, safety, program, state, activity):
    value = interpret(mode, safety, program)
    assert value["equipment_status"] == state
    assert value["activity"] == activity


@pytest.mark.parametrize("prefix", ["", "/api/robot-motion/ligand_ur5e"])
def test_browser_workspace_offline(tmp_path, prefix):
    """Optional real-browser regression; set ROBOT_MOTION_BROWSER_MODULE.

    The module is a locally installed Playwright package. No dependency download,
    real observer, live service or remote browser request is permitted here.
    """
    import os
    import socket
    import subprocess
    import threading
    import time

    import uvicorn
    from fastapi import FastAPI, Request, Response

    if not os.environ.get("ROBOT_MOTION_BROWSER_MODULE"):
        pytest.skip("Set ROBOT_MOTION_BROWSER_MODULE to a local Playwright module")
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen()
    base = "http://127.0.0.1:" + str(listener.getsockname()[1])
    app = create_app(Settings(driver="ur", model="ur5e", observe=False))
    if prefix:
        # Mirror the dashboard's transport contract without authentication or
        # live upstreams. Its actual session gate is tested in the dashboard repo.
        dashboard = FastAPI()
        reads = {"web/index.html", "web/main.js", "web/style.css",
                 "web/pyxarm/style.css", "web/pyxarm/cytoscape.min.js",
                 "status", "drivers", "graph"}

        @dashboard.middleware("http")
        async def proxy_contract(request: Request, call_next):
            path = request.url.path.removeprefix(prefix + "/")
            allowed = reads if request.method == "GET" else {"graph/validate", "graph/preview"}
            if request.url.query or path not in allowed:
                return Response(status_code=404)
            response = await call_next(request)
            response.headers["Content-Security-Policy"] = (
                "default-src 'none'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
                "connect-src 'self'; img-src 'self' data: blob:; font-src 'self'; "
                "frame-ancestors 'self'; base-uri 'none'; form-action 'none'"
            )
            response.headers["Cache-Control"] = "no-store"
            return response

        dashboard.mount(prefix, app)
        app = dashboard
    server = uvicorn.Server(uvicorn.Config(app, log_level="error", loop="asyncio", ws="none"))
    worker = threading.Thread(target=server.run, kwargs={"sockets": [listener]}, daemon=True)
    worker.start()
    try:
        deadline = time.monotonic() + 5
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.02)
        assert server.started
        completed = subprocess.run(
            [os.getenv("ROBOT_MOTION_NODE", "node"), "-e", _BROWSER_WORKSPACE_TEST,
             base + prefix, str(tmp_path)], capture_output=True, text=True, timeout=90,
        )
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert "workspace checks passed" in completed.stdout
    finally:
        server.should_exit = True
        worker.join(timeout=10)
        listener.close()
        assert not worker.is_alive()


_BROWSER_WORKSPACE_TEST = r"""
const assert = require('node:assert/strict');
const fs = require('node:fs');
const {chromium} = require(process.env.ROBOT_MOTION_BROWSER_MODULE);
const base = process.argv[1], output = process.argv[2];
(async () => {
  const browser = await chromium.launch({headless: true});
  try {
    const context = await browser.newContext({viewport: {width: 1440, height: 1000}});
    const page = await context.newPage();
    const errors = [], forbidden = [], sockets = [];
    page.on('pageerror', e => errors.push(e.message));
    await context.addInitScript(() => {
      window.cspViolations = [];
      document.addEventListener('securitypolicyviolation', event => {
        window.cspViolations.push(event.violatedDirective + ': ' + event.blockedURI);
      });
    });
    page.on('websocket', ws => sockets.push(ws.url()));
    await context.route('**/*', route => {
      const request = route.request(), url = new URL(request.url());
      const reads = ['/web/index.html', '/web/main.js', '/web/style.css', '/web/pyxarm/style.css',
        '/web/pyxarm/cytoscape.min.js', '/status', '/drivers', '/graph', '/favicon.ico'];
      const root = new URL(base), prefix = root.pathname.replace(/\/$/, '');
      const path = url.pathname.startsWith(prefix + '/') ? url.pathname.slice(prefix.length) : '';
      const allowed = url.origin === root.origin && !url.search &&
        (request.method() === 'GET' ? reads.includes(path) :
         request.method() === 'POST' && ['/graph/validate', '/graph/preview'].includes(path));
      if (!allowed) { forbidden.push(request.method() + ' ' + url.pathname); return route.abort(); }
      return route.continue();
    });
    await page.goto(base + '/web/index.html', {waitUntil: 'networkidle'});
    await page.waitForFunction(() => document.querySelector('#result').textContent.includes('Topology valid'));
    assert.equal(await page.locator('#graph-workspace').isVisible(), false);
    assert.equal(await page.locator('#control-workspace').isVisible(), true);
    const refreshed = page.waitForResponse(r => r.url().endsWith('/status') && r.request().method() === 'GET');
    await page.locator('#refresh-status').click();
    await refreshed;
    await page.screenshot({path: output + '/control-desktop.png', fullPage: true});
    await page.locator('#tab-direct').click();
    assert.equal(await page.locator('.jog-xy-pad').isVisible(), true);
    assert.equal(await page.locator('#live-joints input').count(), 6);
    await page.screenshot({path: output + '/control-direct.png', fullPage: true});
    await page.locator('#tab-graph').click();
    await page.locator('#open-workspace').click();
    assert.equal(await page.locator('[data-hardware]:not(:disabled)').count(), 0);
    assert.equal(await page.locator('#graph-model').inputValue(), 'ur5e');
    assert.equal(await page.locator('#draft-joints input').count(), 6);
    assert.equal(await page.locator('#draft-joints input').first().inputValue(), '');
    assert.equal(await page.locator('#live-joints input').first().inputValue(), 'Not observed');
    async function addNode(id, offset) {
      await page.locator('#reset-node').click();
      await page.locator('#node-id').fill(id);
      for (let i = 0; i < 6; i++) await page.locator('#draft-joints-' + i).fill(String(i + offset));
      await page.locator('#save-node').click();
      await page.waitForFunction(id => document.querySelector('#graph-input').value.includes('"id": "' + id + '"'), id);
    }
    await addNode('home', 0);
    await addNode('target', 10);
    await page.locator('#edge-source').selectOption('home');
    await page.locator('#edge-target').selectOption('target');
    await page.locator('#save-edge').click();
    await page.waitForFunction(() => document.querySelector('#graph-summary').textContent.includes('1 directed edges'));
    await page.locator('#source').selectOption('home');
    await page.locator('#target').selectOption('target');
    await page.locator('#preview').click();
    await page.waitForFunction(() => document.querySelector('#result').textContent.includes('Preview: home → target'));
    assert.match(await page.locator('#result').innerText(), /Executed: NO/);
    await page.screenshot({path: output + '/workspace-desktop.png', fullPage: true});
    const downloadPromise = page.waitForEvent('download');
    await page.locator('#export-graph').click();
    const download = await downloadPromise;
    await download.saveAs(output + '/draft.json');
    const draft = JSON.parse(fs.readFileSync(output + '/draft.json', 'utf8'));
    assert.equal(draft.nodes.length, 2);
    assert.equal(draft.edges.length, 1);
    // Reverse direction is not invented by the graph editor or planner.
    await page.locator('#source').selectOption('target');
    await page.locator('#target').selectOption('home');
    await page.locator('#preview').click();
    await page.waitForFunction(() => document.querySelector('#result').dataset.error === 'true');
    assert.match(await page.locator('#result').innerText(), /HTTP 422/);
    // Linear edges refuse a missing explicit TCP; failed edits preserve draft.
    await page.locator('#edge-source').selectOption('target');
    await page.locator('#edge-target').selectOption('home');
    await page.locator('#edge-mode').selectOption('linear');
    await page.locator('#save-edge').click();
    await page.waitForFunction(() => document.querySelector('#result').textContent.includes('explicit TCP pose'));
    assert.equal(JSON.parse(await page.locator('#graph-input').inputValue()).edges.length, 1);
    // Delete is local, removes incident edges, and is undoable.
    await page.locator('#node-select').selectOption('target');
    await page.locator('#delete-node').click();
    await page.waitForFunction(() => document.querySelector('#graph-summary').textContent.includes('1 nodes · 0 directed'));
    await page.locator('#undo').click();
    await page.waitForFunction(() => document.querySelector('#graph-summary').textContent.includes('2 nodes · 1 directed'));
    // A delayed route reply cannot overwrite a different route selection.
    let release, seen;
    const gate = new Promise(resolve => { release = resolve; });
    const hit = new Promise(resolve => { seen = resolve; });
    await page.route('**/graph/preview', async route => { seen(); await gate; await route.continue(); });
    await page.locator('#source').selectOption('home');
    await page.locator('#target').selectOption('target');
    await page.locator('#preview').click();
    await hit;
    await page.locator('#target').selectOption('home');
    const response = page.waitForResponse(r => r.url().endsWith('/graph/preview'));
    release(); await response;
    await page.evaluate(() => new Promise(resolve => setTimeout(resolve, 100)));
    assert.match(await page.locator('#result').innerText(), /Route selection changed/);
    await page.unroute('**/graph/preview');
    // Invalid JSON disables stale preview/export. Selectors are keyboard usable.
    await page.locator('.rm-json summary').click();
    await page.locator('#graph-input').fill('{broken');
    assert.equal(await page.locator('#preview').isDisabled(), true);
    assert.equal(await page.locator('#export-graph').isDisabled(), true);
    await page.locator('#validate').click();
    assert.equal(await page.locator('#save-node').isDisabled(), true);
    await page.locator('#graph-input').fill(JSON.stringify(draft));
    await page.locator('#validate').click();
    await page.waitForFunction(() => !document.querySelector('#export-graph').disabled);
    // A late validation response cannot replace newer raw JSON.
    let releaseValidation, sawValidation;
    const validationGate = new Promise(resolve => { releaseValidation = resolve; });
    const validationHit = new Promise(resolve => { sawValidation = resolve; });
    await page.route('**/graph/validate', async route => { sawValidation(); await validationGate; await route.continue(); });
    await page.locator('#validate').click();
    await validationHit;
    await page.locator('#graph-input').fill('{newer unfinished draft');
    releaseValidation();
    await page.waitForFunction(() => !document.querySelector('#validate').disabled);
    assert.equal(await page.locator('#graph-input').inputValue(), '{newer unfinished draft');
    assert.equal(await page.locator('#export-graph').isDisabled(), true);
    await page.unroute('**/graph/validate');
    // Import uses real server validation and generates the correct model fields.
    const mg = {schema_version:'robot-motion/1', robot_model:'mg400', nodes:[{id:'draft_a', joints_deg:[1,2,3,4]}], edges:[]};
    await page.locator('#graph-file').setInputFiles({name:'mg.json', mimeType:'application/json', buffer:Buffer.from(JSON.stringify(mg))});
    await page.waitForFunction(() => document.querySelector('#graph-model').value === 'mg400');
    assert.equal(await page.locator('#draft-joints input').count(), 4);
    assert.equal(await page.locator('#live-joints input').count(), 6);
    assert.match(await page.locator('#model-note').innerText(), /differs from the configured robot profile/);
    await page.locator('#graph-file').setInputFiles({name:'large.json', mimeType:'application/json', buffer:Buffer.alloc(262145,32)});
    await page.waitForFunction(() => document.querySelector('#result').textContent.includes('exceeds 256 KiB'));
    assert.equal(await page.locator('#preview').isDisabled(), true);
    // Re-import a useful fixture for responsive checks; no actual robot data.
    await page.locator('#graph-file').setInputFiles({name:'draft.json', mimeType:'application/json', buffer:Buffer.from(JSON.stringify(draft))});
    await page.waitForFunction(() => !document.querySelector('#export-graph').disabled);
    await page.locator('#theme-toggle').click();
    assert.equal(await page.locator('html').evaluate(e => e.classList.contains('dark')), true);
    await page.screenshot({path: output + '/workspace-dark.png', fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
    const fits = await page.locator('#cy').evaluate(element => {
      const graph = element._cyreg.cy;
      const box = graph.nodes().renderedBoundingBox();
      return box.x1 >= 0 && box.y1 >= 0 && box.x2 <= element.clientWidth && box.y2 <= element.clientHeight;
    });
    assert.equal(fits, true, 'graph refits inside the mobile canvas');
    await page.locator('#close-workspace').click();
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
    await page.locator('#open-workspace').click();
    assert.equal(JSON.parse(await page.locator('#graph-input').inputValue()).nodes.length, 2);
    await page.locator('#close-workspace').click();
    await page.locator('#tab-graph').focus();
    await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('#tab-direct').getAttribute('aria-selected'), 'true');
    assert.equal(await page.locator('[data-hardware]:not(:disabled)').count(), 0);
    await page.screenshot({path: output + '/workspace-mobile.png', fullPage:true});
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
    // Receive-only RTDE snapshots populate measured fields, never the draft or
    // hardware controls. A subsequent missing/invalid sample clears them.
    const draftBeforeTelemetry = await page.locator('#graph-input').inputValue();
    await page.route('**/status', route => route.fulfill({
      status:200, contentType:'application/json', body:JSON.stringify({
        equipment_name:'Offline RTDE fixture', equipment_status:'ready', allowed_actions:[],
        details:{model:'ur5e', control_enabled:false, observation_enabled:true,
          robotmode:'RUNNING', safetystatus:'NORMAL', program_state:'STOPPED',
          observed_time:'2026-01-01T12:00:00Z',
          telemetry:{valid:true, source:'rtde_receive', joints_deg:[0,1,2,3,4,5],
            tcp_mm_rpy_deg:[100,200,300,10,20,30]}},
      }),
    }));
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => document.querySelector('#live-joints-0').value === '0.000');
    assert.equal(await page.locator('#live-tcp-0').inputValue(), '100.000');
    assert.equal(await page.locator('#live-tcp-5').inputValue(), '30.000');
    assert.equal(await page.locator('#graph-input').inputValue(), draftBeforeTelemetry);
    assert.equal(await page.locator('[data-hardware]:not(:disabled)').count(), 0);
    assert.match(await page.locator('#telemetry-note').innerText(), /RTDE receive-only snapshot/);
    await page.unroute('**/status');
    // Even a misleading capability payload cannot activate this UI scaffold.
    await page.route('**/status', route => route.fulfill({
      status:200, contentType:'application/json', body:JSON.stringify({
        equipment_name:'Offline test fixture', equipment_status:'ready',
        allowed_actions:['graph.move_to', 'connect'],
        details:{model:'ur5e', control_enabled:true, robotmode:'RUNNING',
          safetystatus:'NORMAL', program_state:'STOPPED'},
      }),
    }));
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => document.querySelector('#controller').textContent === 'RUNNING');
    await page.waitForFunction(() => document.querySelector('#live-joints-0').value === 'Not observed');
    assert.equal(await page.locator('#live-tcp-0').inputValue(), 'Not observed');
    assert.equal(await page.locator('[data-hardware]:not(:disabled)').count(), 0);
    await page.unroute('**/status');
    await page.route('**/status', route => route.fulfill({status:503, body:'Offline fixture unavailable'}));
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => document.querySelector('#controller').textContent === 'Unknown');
    assert.equal(await page.locator('#safety').innerText(), 'Unknown');
    assert.equal(await page.locator('#program').innerText(), 'Unknown');
    assert.equal(await page.locator('#live-tcp-0').inputValue(), 'Not observed');
    assert.equal(await page.locator('[data-hardware]:not(:disabled)').count(), 0);
    assert.deepEqual(errors, []);
    assert.deepEqual(await page.evaluate(() => window.cspViolations), []);
    assert.deepEqual(forbidden, []);
    assert.deepEqual(sockets, []);
    console.log('workspace checks passed');
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
"""
