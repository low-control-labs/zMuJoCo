const std = @import("std");
const c = @import("lib_z_mujoco").c;
const rollout = @import("rollout").rollout;
const testing = std.testing;

// Port of python/mujoco/rollout_test.py. The rollout results are compared
// exactly against a plain step-by-step reference (py_rollout in the Python test).
// Not ported: the infer_nbatch_* tests, which test Python's broadcasting of
// single-row inputs; the Zig rollout takes full-size arrays.

//-------------------------------- models used for testing --------------------------------------

const test_xml =
    \\<mujoco>
    \\  <worldbody>
    \\    <light pos="0 0 2"/>
    \\    <geom type="plane" size="5 5 .1"/>
    \\    <body pos="0 0 .1">
    \\      <joint name="yaw" axis="0 0 1"/>
    \\      <joint name="pitch" axis="0 1 0"/>
    \\      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
    \\      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
    \\      <site name="site" pos="1 0 0"/>
    \\    </body>
    \\  </worldbody>
    \\  <actuator>
    \\    <general joint="pitch" gainprm="100"/>
    \\    <general joint="yaw" dyntype="filter" dynprm="1" gainprm="100"/>
    \\  </actuator>
    \\  <sensor>
    \\    <accelerometer site="site"/>
    \\  </sensor>
    \\</mujoco>
;

const test_xml_no_sensors =
    \\<mujoco>
    \\  <worldbody>
    \\    <light pos="0 0 2"/>
    \\    <geom type="plane" size="5 5 .1"/>
    \\    <body pos="0 0 .1">
    \\      <joint name="yaw" axis="0 0 1"/>
    \\      <joint name="pitch" axis="0 1 0"/>
    \\      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
    \\      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
    \\      <site name="site" pos="1 0 0"/>
    \\    </body>
    \\  </worldbody>
    \\  <actuator>
    \\    <general joint="pitch" gainprm="100"/>
    \\    <general joint="yaw" dyntype="filter" dynprm="1" gainprm="100"/>
    \\  </actuator>
    \\</mujoco>
;

const test_xml_no_actuators =
    \\<mujoco>
    \\  <worldbody>
    \\    <light pos="0 0 2"/>
    \\    <geom type="plane" size="5 5 .1"/>
    \\    <body pos="0 0 .1">
    \\      <joint name="yaw" axis="0 0 1"/>
    \\      <joint name="pitch" axis="0 1 0"/>
    \\      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
    \\      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
    \\      <site name="site" pos="1 0 0"/>
    \\    </body>
    \\  </worldbody>
    \\  <sensor>
    \\    <accelerometer site="site"/>
    \\  </sensor>
    \\</mujoco>
;

const test_xml_mocap =
    \\<mujoco>
    \\  <worldbody>
    \\    <body name="1" mocap="true">
    \\    </body>
    \\    <body name="2" mocap="true">
    \\    </body>
    \\  </worldbody>
    \\  <sensor>
    \\    <framepos objtype="xbody" objname="1"/>
    \\    <framequat objtype="xbody" objname="1"/>
    \\  </sensor>
    \\</mujoco>
;

const test_xml_empty =
    \\<mujoco>
    \\</mujoco>
;

const test_xml_diverge =
    \\<mujoco>
    \\  <option>
    \\    <flag gravity="disable"/>
    \\  </option>
    \\  <worldbody>
    \\    <geom type="plane" size="5 5 .1"/>
    \\    <body pos="0 0 -.3" euler="30 45 90">
    \\      <freejoint/>
    \\      <geom type="box" size=".1 .2 .4"/>
    \\    </body>
    \\  </worldbody>
    \\  <keyframe>
    \\    <key name="non-diverging" qpos="0 0 .5 1 0 0 0"/>
    \\  </keyframe>
    \\</mujoco>
;

const all_models = [_][]const u8{ test_xml, test_xml_no_sensors, test_xml_no_actuators, test_xml_empty };

//--------------------------------------- helpers -----------------------------------------------

fn loadString(xml: []const u8) ![*c]c.mjModel {
    const z = try testing.allocator.dupeSentinel(u8, xml, 0);
    defer testing.allocator.free(z);
    var err: [1000]u8 = undefined;
    const spec = c.mj_parseXMLString(z, null, &err, err.len);
    if (spec == null) return error.ModelParseFailed;
    defer c.mj_deleteSpec(spec);
    const m = c.mj_compile(spec, null);
    if (m == null) return error.ModelCompileFailed;
    return m;
}

fn nstateOf(m: [*c]const c.mjModel) usize {
    return @intCast(c.mj_stateSize(m, c.mjSTATE_FULLPHYSICS));
}

fn randn(random: std.Random, out: []f64) void {
    for (out) |*x| x.* = random.floatNorm(f64);
}

// rollout outputs, owned
const Out = struct {
    state: []f64,
    sensordata: []f64,
    fn init(m: [*c]const c.mjModel, nbatch: usize, nstep: usize) !Out {
        return .{
            .state = try testing.allocator.alloc(f64, nbatch * nstep * nstateOf(m)),
            .sensordata = try testing.allocator.alloc(f64, nbatch * nstep * @as(usize, @intCast(m.*.nsensordata))),
        };
    }
    fn deinit(o: Out) void {
        testing.allocator.free(o.state);
        testing.allocator.free(o.sensordata);
    }
    fn expectEqual(a: Out, b: Out) !void {
        try testing.expectEqualSlices(f64, a.state, b.state);
        try testing.expectEqualSlices(f64, a.sensordata, b.sensordata);
    }
};

// step / one_rollout / py_rollout of the Python test: the plain reference
fn pyRollout(models: []const [*c]c.mjModel, d: [*c]c.mjData, nbatch: usize, nstep: usize, initial_state: []const f64, control: []const f64, control_spec: c_int, out: Out) void {
    const nstate = nstateOf(models[0]);
    const ncontrol: usize = @intCast(c.mj_stateSize(models[0], control_spec));
    const nsensordata: usize = @intCast(models[0].*.nsensordata);
    for (0..nbatch) |r| {
        const m = if (models.len == 1) models[0] else models[r];
        c.mj_resetData(m, d);
        for (0..nstep) |t| {
            const step = r * nstep + t;
            if (t == 0) c.mj_setState(m, d, initial_state[r * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
            c.mj_setState(m, d, control[step * ncontrol ..].ptr, control_spec);
            c.mj_step(m, d);
            c.mj_getState(m, d, out.state[step * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
            @memcpy(out.sensordata[step * nsensordata ..][0..nsensordata], d.*.sensordata[0..nsensordata]);
        }
    }
}

// rollout vs reference for random initial states and controls
fn compareWithReference(xml: []const u8, nbatch: usize, nstep: usize) !void {
    var prng: std.Random.DefaultPrng = .init(42);
    const random = prng.random();
    const m = try loadString(xml);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const nu: usize = @intCast(m.*.nu);
    const initial_state = try testing.allocator.alloc(f64, nbatch * nstateOf(m));
    defer testing.allocator.free(initial_state);
    const control = try testing.allocator.alloc(f64, nbatch * nstep * nu);
    defer testing.allocator.free(control);
    randn(random, initial_state);
    randn(random, control);

    const out = try Out.init(m, nbatch, nstep);
    defer out.deinit();
    try rollout(&.{m}, &.{d}, nstep, initial_state, .{ .control = control, .state = out.state, .sensordata = out.sensordata });

    const ref = try Out.init(m, nbatch, nstep);
    defer ref.deinit();
    pyRollout(&.{m}, d, nbatch, nstep, initial_state, control, c.mjSTATE_CTRL, ref);
    try out.expectEqual(ref);
}

//--------------------------------- test basic operation ----------------------------------------

test "single step" {
    for (all_models) |xml| try compareWithReference(xml, 1, 1);
}

test "one rollout" {
    for (all_models) |xml| try compareWithReference(xml, 1, 3);
}

test "multi step" {
    for (all_models) |xml| try compareWithReference(xml, 5, 1);
}

test "multi rollout" {
    for (all_models) |xml| try compareWithReference(xml, 2, 3);
}

test "multi model" {
    // like the Python test: the first body shifted by i in each model
    const nbatch = 3;
    const nstep = 3;
    for (all_models) |xml| {
        var prng: std.Random.DefaultPrng = .init(42);
        const random = prng.random();

        var models: [nbatch][*c]c.mjModel = undefined;
        for (0..nbatch) |i| {
            var pos_buf: [64]u8 = undefined;
            const pos = try std.fmt.bufPrint(&pos_buf, "<body pos=\"{d} {d} {d}\"", .{ i, i, 0.1 + @as(f64, @floatFromInt(i)) });
            const shifted = try std.mem.replaceOwned(u8, testing.allocator, xml, "<body pos=\"0 0 .1\"", pos);
            defer testing.allocator.free(shifted);
            models[i] = try loadString(shifted);
        }
        defer for (models) |m| c.mj_deleteModel(m);
        var const_models: [nbatch][*c]const c.mjModel = undefined;
        for (&const_models, models) |*cm, m| cm.* = m;
        const d = c.mj_makeData(models[0]);
        defer c.mj_deleteData(d);

        const nu: usize = @intCast(models[0].*.nu);
        const initial_state = try testing.allocator.alloc(f64, nbatch * nstateOf(models[0]));
        defer testing.allocator.free(initial_state);
        const control = try testing.allocator.alloc(f64, nbatch * nstep * nu);
        defer testing.allocator.free(control);
        randn(random, initial_state);
        randn(random, control);

        const out = try Out.init(models[0], nbatch, nstep);
        defer out.deinit();
        try rollout(&const_models, &.{d}, nstep, initial_state, .{ .control = control, .state = out.state, .sensordata = out.sensordata });

        const ref = try Out.init(models[0], nbatch, nstep);
        defer ref.deinit();
        pyRollout(&models, d, nbatch, nstep, initial_state, control, c.mjSTATE_CTRL, ref);
        try out.expectEqual(ref);
    }
}

test "detect divergence" {
    const m = try loadString(test_xml_diverge);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);
    const nstate = nstateOf(m);

    // diverging (0, 2) and non-diverging (1, 3) initial states
    const nbatch = 4;
    const initial_state = try testing.allocator.alloc(f64, nbatch * nstate);
    defer testing.allocator.free(initial_state);
    c.mj_getState(m, d, initial_state[0 * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
    c.mj_getState(m, d, initial_state[2 * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
    c.mj_resetDataKeyframe(m, d, 0); // keyframe 0 does not diverge
    c.mj_getState(m, d, initial_state[1 * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
    c.mj_getState(m, d, initial_state[3 * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);

    const nstep = 10000; // divergence after ~15s, timestep = 2e-3
    const state = try testing.allocator.alloc(f64, nbatch * nstep * nstate);
    defer testing.allocator.free(state);
    var prng: std.Random.DefaultPrng = .init(42);
    randn(prng.random(), state);
    try rollout(&.{m}, &.{d}, nstep, initial_state, .{ .state = state });

    // time (state element 0) of the last two steps of rollout r
    const last = struct {
        fn time(s: []const f64, r: usize, back: usize, n: usize, ns: usize) f64 {
            return s[(r * n + n - back) * ns];
        }
    }.time;

    // diverged: final timesteps are identical
    try testing.expectEqual(last(state, 0, 1, nstep, nstate), last(state, 0, 2, nstep, nstate));
    try testing.expectEqual(last(state, 2, 1, nstep, nstate), last(state, 2, 2, nstep, nstate));
    // did not diverge: final timesteps are different
    try testing.expect(last(state, 1, 1, nstep, nstate) != last(state, 1, 2, nstep, nstate));
    try testing.expect(last(state, 3, 1, nstep, nstate) != last(state, 3, 2, nstep, nstate));
}

//-------------------------------- test threaded operation --------------------------------------

test "threading" {
    // threaded results match the single-threaded ones (test_threading_native)
    const m = try loadString(test_xml);
    defer c.mj_deleteModel(m);
    const nthread = 4;
    var datas: [nthread][*c]c.mjData = undefined;
    for (&datas) |*d| d.* = c.mj_makeData(m);
    defer for (datas) |d| c.mj_deleteData(d);

    const nbatch = 100;
    const nstep = 5;
    var prng: std.Random.DefaultPrng = .init(42);
    const initial_state = try testing.allocator.alloc(f64, nbatch * nstateOf(m));
    defer testing.allocator.free(initial_state);
    randn(prng.random(), initial_state);

    const single = try Out.init(m, nbatch, nstep);
    defer single.deinit();
    try rollout(&.{m}, datas[0..1], nstep, initial_state, .{ .state = single.state, .sensordata = single.sensordata });

    const threaded = try Out.init(m, nbatch, nstep);
    defer threaded.deinit();
    try rollout(&.{m}, &datas, nstep, initial_state, .{ .state = threaded.state, .sensordata = threaded.sensordata });
    try threaded.expectEqual(single);

    // and with an explicit chunk size
    try rollout(&.{m}, &datas, nstep, initial_state, .{ .state = threaded.state, .sensordata = threaded.sensordata, .chunk_size = 7 });
    try threaded.expectEqual(single);
}

test "warmstart" {
    const m = try loadString(test_xml);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);
    const nstate = nstateOf(m);
    const nu: usize = @intCast(m.*.nu);
    const nv: usize = @intCast(m.*.nv);

    const control = try testing.allocator.alloc(f64, nu);
    defer testing.allocator.free(control);
    @memset(control, 0);
    const state0 = try testing.allocator.alloc(f64, nstate);
    defer testing.allocator.free(state0);
    @memset(state0, 0);
    const state1 = try testing.allocator.alloc(f64, nstate);
    defer testing.allocator.free(state1);
    const state2 = try testing.allocator.alloc(f64, nstate);
    defer testing.allocator.free(state2);

    // take one step, save the state and qacc_warmstart
    c.mj_setState(m, d, state0.ptr, c.mjSTATE_FULLPHYSICS);
    c.mj_setState(m, d, control.ptr, c.mjSTATE_CTRL);
    c.mj_step(m, d);
    c.mj_getState(m, d, state1.ptr, c.mjSTATE_FULLPHYSICS);
    const warmstart = try testing.allocator.dupe(f64, d.*.qacc_warmstart[0..nv]);
    defer testing.allocator.free(warmstart);

    // take one more step (uses the correct warmstart)
    c.mj_setState(m, d, control.ptr, c.mjSTATE_CTRL);
    c.mj_step(m, d);
    c.mj_getState(m, d, state2.ptr, c.mjSTATE_FULLPHYSICS);

    // rollout without warmstart: not exact
    const state = try testing.allocator.alloc(f64, nstate);
    defer testing.allocator.free(state);
    try rollout(&.{m}, &.{d}, 1, state1, .{ .control = control, .state = state });
    try testing.expect(!std.mem.eql(f64, state, state2));

    // rollout with warmstart: exact
    try rollout(&.{m}, &.{d}, 1, state1, .{ .control = control, .state = state, .initial_warmstart = warmstart });
    try testing.expectEqualSlices(f64, state2, state);
}

test "mocap" {
    const m = try loadString(test_xml_mocap);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const initial_state = try testing.allocator.alloc(f64, nstateOf(m));
    defer testing.allocator.free(initial_state);
    @memset(initial_state, 0);

    const pos1 = [3]f64{ 1, 2, 3 };
    const pos2 = [3]f64{ 2, 3, 4 };
    var quat1 = [4]f64{ 1, 2, 3, 4 };
    var quat2 = [4]f64{ 2, 3, 4, 5 };
    for ([_]*[4]f64{ &quat1, &quat2 }) |q| {
        const norm = @sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (q) |*v| v.* /= norm;
    }
    const control = pos1 ++ pos2 ++ quat1 ++ quat2;
    const control_spec = c.mjSTATE_MOCAP_POS | c.mjSTATE_MOCAP_QUAT;

    const sensordata = try testing.allocator.alloc(f64, @intCast(m.*.nsensordata));
    defer testing.allocator.free(sensordata);
    try rollout(&.{m}, &.{d}, 1, initial_state, .{ .control = &control, .control_spec = control_spec, .sensordata = sensordata });

    for (0..3) |i| try testing.expectApproxEqAbs(pos1[i], sensordata[i], 1.5e-6);
    for (0..4) |i| try testing.expectApproxEqAbs(quat1[i], sensordata[3 + i], 1.5e-6);
}

//----------------------------------- test correctness ------------------------------------------

test "bad sizes" {
    const m = try loadString(test_xml);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);
    const nstate = nstateOf(m);
    const nu: usize = @intCast(m.*.nu);

    var buf: [1024]f64 = @splat(0);
    const state = buf[0 .. 2 * nstate];

    try testing.expectError(error.NoData, rollout(&.{m}, &.{}, 1, state, .{}));
    try testing.expectError(error.ModelCount, rollout(&.{}, &.{d}, 1, state, .{}));
    try testing.expectError(error.ModelCount, rollout(&.{ m, m, m }, &.{d}, 1, state, .{}));
    try testing.expectError(error.InitialStateSize, rollout(&.{m}, &.{d}, 1, buf[0 .. nstate + 1], .{}));
    try testing.expectError(error.ControlSize, rollout(&.{m}, &.{d}, 3, state, .{ .control = buf[0 .. 2 * 3 * nu - 1] }));
    try testing.expectError(error.WarmstartSize, rollout(&.{m}, &.{d}, 1, state, .{ .initial_warmstart = buf[0..1] }));
    try testing.expectError(error.StateSize, rollout(&.{m}, &.{d}, 3, state, .{ .state = buf[0..5] }));
    try testing.expectError(error.SensordataSize, rollout(&.{m}, &.{d}, 3, state, .{ .sensordata = buf[0..5] }));
}

test "stateless" {
    const m = try loadString(test_xml);
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);
    const nu: usize = @intCast(m.*.nu);
    const nv: usize = @intCast(m.*.nv);
    const nbody: usize = @intCast(m.*.nbody);
    const nmocap: usize = @intCast(m.*.nmocap);

    var prng: std.Random.DefaultPrng = .init(42);
    const random = prng.random();
    const nbatch = 3;
    const nstep = 3;
    const initial_state = try testing.allocator.alloc(f64, nbatch * nstateOf(m));
    defer testing.allocator.free(initial_state);
    const control = try testing.allocator.alloc(f64, nbatch * nstep * nu);
    defer testing.allocator.free(control);
    randn(random, initial_state);
    randn(random, control);

    // step with a clean mjData
    const out = try Out.init(m, nbatch, nstep);
    defer out.deinit();
    try rollout(&.{m}, &.{d}, nstep, initial_state, .{ .control = control, .state = out.state, .sensordata = out.sensordata });

    // fill the user fields with random values
    randn(random, d.*.ctrl[0..nu]);
    randn(random, d.*.qfrc_applied[0..nv]);
    randn(random, d.*.xfrc_applied[0 .. 6 * nbody]);
    randn(random, d.*.mocap_pos[0 .. 3 * nmocap]);
    randn(random, d.*.mocap_quat[0 .. 4 * nmocap]);

    // roll out again: still the same outputs
    const out2 = try Out.init(m, nbatch, nstep);
    defer out2.deinit();
    try rollout(&.{m}, &.{d}, nstep, initial_state, .{ .control = control, .state = out2.state, .sensordata = out2.sensordata });
    try out2.expectEqual(out);
}
