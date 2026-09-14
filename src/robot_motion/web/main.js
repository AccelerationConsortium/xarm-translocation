"use strict";
// Reuse pyxarm's visual vocabulary and Cytoscape renderer, not its command
// handlers, claim globals, camera player, hardcoded ports, or robot settings.
(() => {
  const $ = id => document.getElementById(id);
  const apiBase = new URL("../", location.href);
  const MAX_BYTES = 262144;
  const METHODS = new Map([
    ["status", "GET"], ["drivers", "GET"], ["graph", "GET"],
    ["graph/validate", "POST"], ["graph/preview", "POST"],
  ]);
  let models = {}, observedModel = null, currentGraph = null;
  let revision = 0, routeRevision = 0, busy = false, rawDirty = false;
  let selectedNode = null, unsaved = false, polling = false;
  const undo = [];
  let cy = null;

  function result(message, error = false) {
    $("result").textContent = message;
    $("result").dataset.error = String(error);
  }
  function report(error) { result(error.message || String(error), true); }
  function copy(value) { return JSON.parse(JSON.stringify(value)); }
  function emptyGraph(model) {
    return { schema_version: "robot-motion/1", robot_model: model, nodes: [], edges: [] };
  }
  async function request(path, body) {
    const method = body === undefined ? "GET" : "POST";
    if (METHODS.get(path) !== method) throw new Error("Unsupported prototype request");
    const payload = body === undefined ? undefined : JSON.stringify(body);
    if (payload && new TextEncoder().encode(payload).length > MAX_BYTES)
      throw new Error("Graph request exceeds 256 KiB");
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 8000);
    try {
      const response = await fetch(new URL(path, apiBase), {
        method, cache: "no-store", signal: controller.signal,
        headers: body === undefined ? {} : { "Content-Type": "application/json" },
        body: payload,
      });
      if (!response.ok) {
        const text = (await response.text()).slice(0, 2000);
        throw new Error("HTTP " + response.status + ": " + text);
      }
      return await response.json();
    } finally { clearTimeout(timer); }
  }
  function inputs(container, labels, values = [], readonly = false) {
    const root = $(container);
    root.replaceChildren();
    labels.forEach((label, i) => {
      const column = document.createElement("div");
      column.className = "joint-col";
      const caption = document.createElement("label");
      const id = container + "-" + i;
      caption.htmlFor = id;
      caption.textContent = label;
      const input = document.createElement("input");
      input.id = id;
      input.type = readonly ? "text" : "number";
      input.step = "any";
      input.required = !readonly;
      input.disabled = readonly;
      input.value = readonly ? "Not observed" : (values[i] ?? "");
      input.placeholder = readonly ? "" : "Required";
      column.append(caption, input);
      root.append(column);
    });
  }
  function numberValues(container) {
    return Array.from($(container).querySelectorAll("input")).map(input => {
      if (!input.value.trim() || !Number.isFinite(input.valueAsNumber))
        throw new Error("Enter every coordinate as a finite number");
      return input.valueAsNumber;
    });
  }
  function optionList(id, rows, placeholder = null) {
    const select = $(id), previous = select.value;
    select.replaceChildren();
    if (placeholder !== null) select.add(new Option(placeholder, ""));
    for (const [value, label] of rows) select.add(new Option(label, value));
    if (Array.from(select.options).some(option => option.value === previous))
      select.value = previous;
  }
  function updateModelNote() {
    const draft = currentGraph?.robot_model || $("graph-model").value;
    $("model-note").textContent = observedModel && observedModel !== draft
      ? "Draft model differs from the configured robot profile. No coordinates are transferred between models."
      : "This model belongs to the draft only; it does not switch the configured robot.";
  }
  function updateControls() {
    const usable = !!currentGraph && !rawDirty && !busy;
    const count = currentGraph?.nodes.length || 0;
    $("graph-model").disabled = busy || !Object.keys(models).length;
    for (const id of ["new-graph", "open-graph", "validate"]) $(id).disabled = busy;
    $("export-graph").disabled = !usable;
    $("undo").disabled = busy || !undo.length;
    $("preview").disabled = !usable || !count;
    $("source").disabled = $("target").disabled = !usable || !count;
    $("node-fields").disabled = !usable;
    $("edge-fields").disabled = !usable || count < 2;
    $("node-select").disabled = !usable || !count;
    $("edge-select").disabled = !usable || !currentGraph?.edges.length;
    $("delete-node").disabled = !usable || !selectedNode;
    $("delete-edge").disabled = !usable || !currentGraph?.edges.length;
    // This is not capability-gated hardware control. No handler is attached to
    // these buttons, and the service has no corresponding mutation endpoints.
    document.querySelectorAll("[data-hardware]").forEach(button => { button.disabled = true; });
  }
  function clearRoute() {
    routeRevision += 1;
    if (cy) cy.elements().removeClass("preview");
  }
  function invalidate() {
    revision += 1;
    rawDirty = true;
    unsaved = true;
    clearRoute();
    if (cy) cy.elements().remove();
    $("canvas-empty").hidden = false;
    $("canvas-empty").textContent = "Validate the edited JSON to view its topology.";
    $("graph-summary").textContent = "Unvalidated draft · preview and editor paused";
    result("JSON changed. Validate before editing nodes, exporting, or previewing.");
    updateControls();
  }

  // Node/edge presentation follows the pyxarm graph viewer. Deliberately omit
  // its live-current/gripper leaves: this observer does not measure pose.
  function graphStyles() {
    const dark = document.documentElement.classList.contains("dark");
    return [
      { selector: "node", style: {
        "label": "data(label)", "background-color": dark ? "#1e293b" : "#ffffff",
        "border-color": "#0369a1", "border-width": 2, "width": 34, "height": 34,
        "color": dark ? "#e2e8f0" : "#334155", "font-size": 12,
        "text-valign": "bottom", "text-margin-y": 8,
      }},
      { selector: "edge", style: {
        "curve-style": "bezier", "target-arrow-shape": "triangle", "width": 2,
        "line-color": "#94a3b8", "target-arrow-color": "#94a3b8",
        "label": "data(mode)", "font-size": 10, "color": dark ? "#cbd5e1" : "#475569",
        "text-rotation": "autorotate", "text-margin-y": -8,
      }},
      { selector: ".preview", style: {
        "background-color": "#0284c7", "line-color": "#0284c7",
        "target-arrow-color": "#0284c7", "border-color": "#0284c7",
      }},
      { selector: "node:selected", style: { "border-color": "#d97706", "border-width": 4 }},
    ];
  }
  function renderGraph() {
    if (!cy || !currentGraph) return;
    const positions = new Map(cy.nodes().map(n => [n.id(), { ...n.position() }]));
    cy.elements().remove();
    cy.add([
      ...currentGraph.nodes.map(n => ({ data: { id: n.id, label: n.id } })),
      ...currentGraph.edges.map(e => ({
        data: { id: "edge:" + e.source + "->" + e.target,
          source: e.source, target: e.target, mode: e.mode || "joint" },
      })),
    ]);
    if (currentGraph.nodes.length && currentGraph.nodes.every(n => positions.has(n.id))) {
      cy.nodes().forEach(n => n.position(positions.get(n.id())));
    } else if (currentGraph.nodes.length) {
      cy.layout({ name: "breadthfirst", directed: true, padding: 50, spacingFactor: 1.4, animate: false }).run();
    }
    $("canvas-empty").hidden = currentGraph.nodes.length > 0;
    $("canvas-empty").textContent = "Add a node or open a graph to begin.";
    $("graph-summary").textContent = currentGraph.nodes.length + " nodes · " +
      currentGraph.edges.length + " directed edges · " + currentGraph.robot_model + " draft";
  }
  function selectNode(id = "") {
    selectedNode = currentGraph?.nodes.find(n => n.id === id)?.id || null;
    const node = currentGraph?.nodes.find(n => n.id === selectedNode);
    $("node-select").value = selectedNode || "";
    $("node-id").value = node?.id || "";
    $("node-id").readOnly = !!node;
    const count = models[currentGraph?.robot_model]?.joints || 0;
    inputs("draft-joints", Array.from({ length: count }, (_, i) => "J" + (i + 1) + " (°)"), node?.joints_deg);
    $("include-tcp").checked = !!node?.tcp_mm_rpy_deg;
    inputs("draft-tcp", ["X (mm)", "Y (mm)", "Z (mm)", "Roll (°)", "Pitch (°)", "Yaw (°)"], node?.tcp_mm_rpy_deg);
    toggleTcp();
    $("save-node").textContent = node ? "Update draft node" : "Add node to draft";
    if (cy) {
      cy.nodes().unselect();
      if (selectedNode) cy.getElementById(selectedNode).select();
    }
    updateControls();
  }
  function toggleTcp() {
    const enabled = $("include-tcp").checked;
    $("draft-tcp").hidden = !enabled;
    $("draft-tcp").querySelectorAll("input").forEach(input => {
      input.required = enabled;
      input.disabled = !enabled;
    });
  }
  function renderEditor() {
    const nodes = currentGraph.nodes.map(n => [n.id, n.id]);
    for (const id of ["source", "target", "edge-source", "edge-target"]) optionList(id, nodes);
    for (const id of ["target", "edge-target"]) {
      const from = id === "target" ? "source" : "edge-source";
      if (nodes.length > 1 && $(id).value === $(from).value) $(id).selectedIndex = 1;
    }
    optionList("node-select", nodes, "New node…");
    optionList("edge-select", currentGraph.edges.map((e, i) => [
      String(i), e.source + " → " + e.target + " · " + (e.mode || "joint"),
    ]));
    selectNode(selectedNode || "");
    updateModelNote();
  }
  async function acceptGraph(candidate, { history = true, changed = true } = {}) {
    const token = ++revision;
    busy = true;
    clearRoute();
    updateControls();
    try {
      if (!models[candidate?.robot_model]) throw new Error("Choose a supported draft robot model");
      const check = await request("graph/validate", candidate);
      if (token !== revision) return false;
      if (check.valid_topology !== true || check.physical_validation !== false)
        throw new Error("Unexpected topology-validation response");
      if (history && currentGraph) {
        undo.push(copy(currentGraph));
        if (undo.length > 30) undo.shift();
      }
      currentGraph = copy(candidate);
      rawDirty = false;
      unsaved = changed;
      $("graph-model").value = currentGraph.robot_model;
      $("graph-input").value = JSON.stringify(currentGraph, null, 2);
      renderGraph();
      renderEditor();
      result("Topology valid: " + check.nodes + " nodes, " + check.edges +
        " directed edges.\nPhysical validation: NOT performed. Nothing executed or saved on the server.");
      return true;
    } catch (error) {
      if (token === revision) {
        if (!rawDirty && currentGraph) $("graph-model").value = currentGraph.robot_model;
        report(error);
      }
      return false;
    } finally {
      busy = false;
      updateControls();
    }
  }
  async function validateText() {
    // Invalidate before parsing: an invalid import must never leave a previous
    // graph exportable or a previous route looking valid.
    invalidate();
    try {
      const text = $("graph-input").value;
      if (new TextEncoder().encode(text).length > MAX_BYTES) throw new Error("Graph file exceeds 256 KiB");
      await acceptGraph(JSON.parse(text));
    } catch (error) { report(error); }
  }
  function requireDraft() {
    if (!currentGraph || rawDirty || busy) throw new Error("Validate the current graph first");
    return copy(currentGraph);
  }
  async function preview() {
    const graph = requireDraft(), graphToken = revision, token = ++routeRevision;
    const source = $("source").value, target = $("target").value;
    cy?.elements().removeClass("preview");
    result("Computing topology route…");
    try {
      const reply = await request("graph/preview", { graph, source, target });
      if (graphToken !== revision || token !== routeRevision) return;
      if (reply.executed !== false || reply.physical_validation !== false || !Array.isArray(reply.path))
        throw new Error("Unexpected preview response");
      reply.path.forEach((id, i) => {
        cy?.getElementById(id).addClass("preview");
        if (i) cy?.getElementById("edge:" + reply.path[i - 1] + "->" + id).addClass("preview");
      });
      result("Preview: " + reply.path.join(" → ") + "\nExecuted: NO · Physical validation: NOT performed");
    } catch (error) {
      if (graphToken === revision && token === routeRevision) report(error);
    }
  }

  async function refreshStatus() {
    if (document.hidden || polling) return;
    polling = true;
    try {
      const status = await request("status");
      $("robot-name").textContent = status.equipment_name || "Robot status";
      $("state").textContent = status.equipment_status || "unknown";
      $("status-message").textContent = status.message || "";
      observedModel = status.details?.model || null;
      $("model").textContent = observedModel || "Not configured";
      $("controller").textContent = status.details?.robotmode || "Unknown";
      $("safety").textContent = status.details?.safetystatus || "Unknown";
      $("program").textContent = status.details?.program_state || "Unknown";
    } catch (error) {
      $("state").textContent = "Unknown";
      $("status-message").textContent = "Service unavailable: " + error.message;
      observedModel = null;
      $("model").textContent = "Unknown";
      for (const id of ["controller", "safety", "program"]) $(id).textContent = "Unknown";
    } finally {
      polling = false;
      const count = models[observedModel]?.joints || 0;
      if (count) inputs("live-joints", Array.from({ length: count }, (_, i) => "J" + (i + 1)), [], true);
      else $("live-joints").textContent = "Not observed; robot model unavailable.";
      updateModelNote();
      updateControls();
    }
  }

  function setTheme(dark) {
    document.documentElement.classList.toggle("dark", dark);
    $("theme-toggle").setAttribute("aria-pressed", String(dark));
    if (cy) cy.style(graphStyles());
  }
  $("theme-toggle").addEventListener("click", () => {
    const dark = !document.documentElement.classList.contains("dark");
    setTheme(dark);
    try { localStorage.setItem("theme", dark ? "dark" : "light"); }
    catch (error) { result("Theme changed for this tab; preference could not be stored."); }
  });
  function activateTab(name) {
    for (const id of ["graph", "direct"]) {
      const active = id === name;
      $("tab-" + id).classList.toggle("active", active);
      $("tab-" + id).setAttribute("aria-selected", String(active));
      $("tab-" + id).tabIndex = active ? 0 : -1;
      $("pane-" + id).hidden = !active;
    }
  }
  for (const name of ["graph", "direct"]) {
    $("tab-" + name).addEventListener("click", () => activateTab(name));
    $("tab-" + name).addEventListener("keydown", event => {
      if (["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) {
        event.preventDefault();
        const next = event.key === "Home" ? "graph" : event.key === "End" ? "direct" : name === "graph" ? "direct" : "graph";
        activateTab(next);
        $("tab-" + next).focus();
      }
    });
  }
  $("graph-input").addEventListener("input", invalidate);
  $("validate").addEventListener("click", validateText);
  $("open-graph").addEventListener("click", () => $("graph-file").click());
  $("graph-file").addEventListener("change", async event => {
    const file = event.target.files[0];
    if (!file) return;
    invalidate();
    const token = revision;
    try {
      if (file.size > MAX_BYTES) throw new Error("Graph file exceeds 256 KiB");
      const text = await file.text();
      if (token !== revision) return;
      $("graph-input").value = text;
      await validateText();
    } catch (error) { if (token === revision) report(error); }
    finally { event.target.value = ""; }
  });
  function startNewGraph() {
    if ((unsaved || currentGraph?.nodes.length) &&
        !window.confirm("Start an empty draft? Export first if needed. Undo can restore the last validated graph.")) {
      if (currentGraph) $("graph-model").value = currentGraph.robot_model;
      return;
    }
    acceptGraph(emptyGraph($("graph-model").value));
  }
  $("new-graph").addEventListener("click", startNewGraph);
  $("graph-model").addEventListener("change", startNewGraph);
  $("undo").addEventListener("click", async () => {
    if (!undo.length) return;
    const previous = undo[undo.length - 1];
    if (await acceptGraph(previous, { history: false })) undo.pop();
    updateControls();
  });
  $("export-graph").addEventListener("click", () => {
    try {
      const graph = requireDraft();
      const url = URL.createObjectURL(new Blob([JSON.stringify(graph, null, 2) + "\n"], { type: "application/json" }));
      const link = document.createElement("a");
      link.href = url;
      link.download = "robot-motion-" + graph.robot_model + "-draft.json";
      document.body.append(link);
      link.click();
      link.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      unsaved = false;
      result("Draft exported. It is not commissioned or authorized for hardware execution.");
    } catch (error) { report(error); }
  });
  $("preview").addEventListener("click", () => preview().catch(report));
  for (const id of ["source", "target"]) $(id).addEventListener("change", () => {
    clearRoute();
    result("Route selection changed. Preview again; nothing has moved.");
  });
  $("fit-graph").addEventListener("click", () => { if (cy?.nodes().length) cy.fit(undefined, 50); });
  $("node-select").addEventListener("change", () => selectNode($("node-select").value));
  $("reset-node").addEventListener("click", () => selectNode());
  $("include-tcp").addEventListener("change", toggleTcp);
  $("node-form").addEventListener("submit", async event => {
    event.preventDefault();
    try {
      const graph = requireDraft();
      const id = $("node-id").value.trim();
      if (!selectedNode && graph.nodes.some(n => n.id === id)) throw new Error("Node ID already exists; select it to edit");
      const node = { id, joints_deg: numberValues("draft-joints") };
      if ($("include-tcp").checked) node.tcp_mm_rpy_deg = numberValues("draft-tcp");
      if (selectedNode) graph.nodes = graph.nodes.map(n => n.id === selectedNode ? node : n);
      else graph.nodes.push(node);
      if (await acceptGraph(graph)) selectNode(id);
    } catch (error) { report(error); }
  });
  $("delete-node").addEventListener("click", async () => {
    try {
      const graph = requireDraft(), id = selectedNode;
      if (!id) throw new Error("Select a node first");
      graph.nodes = graph.nodes.filter(n => n.id !== id);
      graph.edges = graph.edges.filter(e => e.source !== id && e.target !== id);
      if (await acceptGraph(graph)) selectNode();
    } catch (error) { report(error); }
  });
  $("edge-form").addEventListener("submit", async event => {
    event.preventDefault();
    try {
      const graph = requireDraft();
      graph.edges.push({ source: $("edge-source").value, target: $("edge-target").value, mode: $("edge-mode").value });
      await acceptGraph(graph);
    } catch (error) { report(error); }
  });
  $("delete-edge").addEventListener("click", async () => {
    try {
      const graph = requireDraft();
      const index = Number($("edge-select").value);
      if (!Number.isInteger(index) || !graph.edges[index]) throw new Error("Select an edge first");
      graph.edges.splice(index, 1);
      await acceptGraph(graph);
    } catch (error) { report(error); }
  });
  window.addEventListener("beforeunload", event => {
    if (unsaved) { event.preventDefault(); event.returnValue = ""; }
  });

  async function init() {
    updateControls();
    try {
      const theme = localStorage.getItem("theme");
      setTheme(theme === "dark" || (theme !== "light" && matchMedia("(prefers-color-scheme: dark)").matches));
    } catch (error) { result("Theme preference unavailable; using light theme."); }
    if (typeof window.cytoscape === "function") {
      cy = window.cytoscape({ container: $("cy"), style: graphStyles(), elements: [],
        minZoom: 0.2, maxZoom: 3, selectionType: "single" });
      let resizeFrame = null;
      new ResizeObserver(() => {
        if (resizeFrame !== null) cancelAnimationFrame(resizeFrame);
        resizeFrame = requestAnimationFrame(() => {
          cy.resize();
          if (cy.nodes().length) cy.fit(undefined, 50);
        });
      }).observe($("cy"));
      cy.on("tap", "node", event => { if (!busy && !rawDirty) selectNode(event.target.id()); });
      cy.on("tap", "edge", event => {
        const data = event.target.data();
        const i = currentGraph?.edges.findIndex(e => e.source === data.source && e.target === data.target);
        if (i >= 0) $("edge-select").value = String(i);
      });
    } else {
      $("canvas-empty").textContent = "Graph renderer unavailable; use the node and edge selectors.";
    }
    try {
      const inventory = await request("drivers");
      models = inventory.models;
      optionList("graph-model", Object.keys(models).map(id => [id, id + " · " + models[id].joints + " joints"]));
      for (const [name, driver] of Object.entries(inventory.drivers)) {
        const row = document.createElement("div");
        row.className = "rm-driver";
        const title = document.createElement("strong"); title.textContent = "[" + name + "]";
        const state = document.createElement("span"); state.textContent = driver.implementation + " · " + driver.control;
        const sdk = document.createElement("small"); sdk.textContent = driver.sdk_installed ? "SDK installed" : "SDK not installed";
        row.append(title, state, sdk); $("drivers").append(row);
      }
      await refreshStatus();
      // A late initial graph response must not replace a draft already edited.
      const token = revision;
      const saved = await request("graph");
      if (token === revision) {
        const model = observedModel || "ur5e";
        await acceptGraph(saved.graph || emptyGraph(model), { history: false, changed: false });
      }
    } catch (error) { report(error); }
    updateControls();
    setInterval(refreshStatus, 15000);
  }
  document.addEventListener("visibilitychange", () => { if (!document.hidden) refreshStatus(); });
  init().catch(report);
})();
