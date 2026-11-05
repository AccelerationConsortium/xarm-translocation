// Run with: node --test test/test_graph_shapes.cjs
// The graph panel reads GET /graph in the xArm shape; robot-motion (the UR
// arms) wraps the graph and writes edges {source, target}. normalizeGraph in
// graph.js converts one to the other.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const code = fs.readFileSync(path.join(__dirname, '../src/web/graph.js'), 'utf8');
const source = code.match(/function normalizeGraph\(body\) \{[\s\S]*?\n    \}\n/);
assert.ok(source, 'normalizeGraph not found in graph.js');
const normalizeGraph = vm.runInNewContext(`(${source[0]})`);

const urGraph = JSON.parse(fs.readFileSync(path.join(__dirname, '../deploy/ur3e_gibbie/graph.json'), 'utf8'));

test('the UR service shape gives every node and edge, edges as from/to', () => {
    const data = normalizeGraph({ graph: urGraph, notice: 'not physically validated' });
    assert.deepEqual(data.nodes.map(n => n.id), urGraph.nodes.map(n => n.id));
    assert.equal(data.edges.length, urGraph.edges.length);
    assert.ok(data.edges.length > 0);
    data.edges.forEach((e, i) => {
        assert.equal(e.from, urGraph.edges[i].source);
        assert.equal(e.to, urGraph.edges[i].target);
        assert.equal(e.mode, urGraph.edges[i].mode);
    });
    assert.ok(data.edges.some(e => e.from === 'prep_pick_up_vial' && e.to === 'pick_up_vial'));
    assert.equal(urGraph.edges[0].from, undefined, 'the input is not modified');
});

test('the xArm shape passes through unchanged', () => {
    const xarm = {
        nodes: [{ id: 'home', tags: ['global_home'] }, { id: 'deck' }],
        edges: [{ from: 'home', to: 'deck', mode: 'joint', speed: 20 }],
        gripper_state_catalog: { empty: {} },
    };
    const data = normalizeGraph(xarm);
    assert.deepEqual(JSON.parse(JSON.stringify(data)), xarm);
    assert.equal(data.edges[0], xarm.edges[0]);
});

test('no configured graph reads as no_graph', () => {
    assert.throws(() => normalizeGraph({ graph: null, notice: 'x' }), /no_graph/);
    assert.throws(() => normalizeGraph(null), /no_graph/);
    assert.throws(() => normalizeGraph({ detail: 'nope' }), /no_graph/);
});

test('the UR gripper catalog becomes the panel catalog and node transitions', () => {
    const graph = {
        nodes: [{ id: 'pick', joints_deg: [0], gripper_states: ['vial_open', 'vial_closed'] }, { id: 'home', joints_deg: [0] }],
        edges: [],
        gripper_catalog: { vial_open: { raw: 207 }, vial_closed: { raw: 242, expect_object: true } },
    };
    const data = normalizeGraph({ graph });
    assert.deepEqual(JSON.parse(JSON.stringify(data.gripper_state_catalog)), {
        vial_open: { stroke: 207, intent: 'position' }, vial_closed: { stroke: 242, intent: 'grasp' },
    });
    assert.deepEqual(JSON.parse(JSON.stringify(data.nodes[0].gripper_transitions)), ['vial_open', 'vial_closed']);
    assert.equal(data.nodes[1].gripper_transitions, undefined);
    assert.equal(graph.nodes[0].gripper_transitions, undefined, 'the input is not modified');
});
