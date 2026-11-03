const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
const c = lib_mujoco.c;
const rollout = @import("rollout").rollout;
const Renderer = @import("renderer").Renderer;
const media = @import("media");

// Examples from python/rollout.ipynb: 100 tippe tops with different spin
// speeds, 100 humanoids with open-loop controls, and a threading benchmark.
// Writes rollout_tops.mp4 and rollout_humanoids.mp4 (needs ffmpeg).
// USAGE: zrollout [humanoid modelfile]

const tippe_top =
    \\<mujoco model="tippe top">
    \\  <option integrator="RK4"/>
    \\
    \\  <asset>
    \\    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
    \\     rgb2=".2 .3 .4" width="300" height="300"/>
    \\    <material name="grid" texture="grid" texrepeat="40 40" reflectance=".2"/>
    \\  </asset>
    \\
    \\  <worldbody>
    \\    <geom size="1 1 .01" type="plane" material="grid"/>
    \\    <light pos="0 0 .6"/>
    \\    <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>
    \\    <camera name="distant" pos="0 -.4 .4" xyaxes="1 0 0 0 1 1"/>
    \\    <body name="top" pos="0 0 .02">
    \\      <freejoint name="top"/>
    \\      <site name="top" pos="0 0 0"/>
    \\      <geom name="ball" type="sphere" size=".02" />
    \\      <geom name="stem" type="cylinder" pos="0 0 .02" size="0.004 .008"/>
    \\      <geom name="ballast" type="box" size=".023 .023 0.005"  pos="0 0 -.015"
    \\       contype="0" conaffinity="0" group="3"/>
    \\    </body>
    \\  </worldbody>
    \\
    \\  <sensor>
    \\    <gyro name="gyro" site="top"/>
    \\  </sensor>
    \\
    \\  <keyframe>
    \\    <key name="spinning" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 200" />
    \\  </keyframe>
    \\</mujoco>
;

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const gpa = init.gpa;
    const arena = init.arena.allocator();
    const args = try init.minimal.args.toSlice(arena);
    const humanoid_path: [*:0]const u8 = if (args.len > 1) args[1].ptr else "model/humanoid/humanoid.xml";

    const nthread = try std.Thread.getCpuCount();
    std.log.info("using {d} threads", .{nthread});

    // tippe top model, compiled from the XML string
    var load_error: [1000]u8 = undefined;
    const spec = c.mj_parseXMLString(tippe_top, null, &load_error, load_error.len);
    if (spec == null) {
        std.log.err("could not parse tippe top: {s}", .{std.mem.sliceTo(&load_error, 0)});
        return error.ModelLoadFailed;
    }
    defer c.mj_deleteSpec(spec);
    const top_model = c.mj_compile(spec, null);
    if (top_model == null) return error.ModelCompileFailed;
    defer c.mj_deleteModel(top_model);
    const top_data = c.mj_makeData(top_model);
    defer c.mj_deleteData(top_data);
    const top_datas = try makeDatas(arena, top_model, nthread);
    defer for (top_datas) |d| c.mj_deleteData(d);

    //------------------------ example: different initial states ------------------------

    {
        // 100 tops, the initial spin speed scaled from 0.5 to 1.5 across the batch
        const nbatch = 100;
        c.mj_resetDataKeyframe(top_model, top_data, 0);
        const initial_states = try tiledState(arena, top_model, top_data, nbatch);
        const nstate = initial_states.len / nbatch;
        for (0..nbatch) |b| {
            initial_states[b * nstate + nstate - 1] *= 0.5 + @as(f64, @floatFromInt(b)) / (nbatch - 1);
        }

        const nstep: usize = @intFromFloat(@floor(6 / top_model.*.opt.timestep * 1.5));
        const state = try arena.alloc(f64, nbatch * nstep * nstate);
        const nsensordata: usize = @intCast(top_model.*.nsensordata);
        const sensordata = try arena.alloc(f64, nbatch * nstep * nsensordata);

        const start = std.Io.Timestamp.now(io, .awake);
        try rollout(&.{top_model}, top_datas, nstep, initial_states, .{ .state = state, .sensordata = sensordata });
        std.log.info("tops: rollout time {d:.2} s", .{seconds(io, start)});

        // render all tops at once, transparent
        var opt: c.mjvOption = undefined;
        c.mjv_defaultOption(&opt);
        opt.flags[c.mjVIS_TRANSPARENT] = 1;
        try renderMany(io, gpa, &.{top_model}, top_data, state, nbatch, nstep, 60, null, &opt, null, 480, 640, 60, "rollout_tops.mp4");
    }

    //---------------------------- example: control inputs ------------------------------

    {
        const humanoid_model = c.mj_loadXML(humanoid_path, null, &load_error, load_error.len);
        if (humanoid_model == null) {
            std.log.err("could not load humanoid: {s}", .{std.mem.sliceTo(&load_error, 0)});
            return error.ModelLoadFailed;
        }
        defer c.mj_deleteModel(humanoid_model);
        const humanoid_data = c.mj_makeData(humanoid_model);
        defer c.mj_deleteData(humanoid_data);
        const humanoid_datas = try makeDatas(arena, humanoid_model, nthread);
        defer for (humanoid_datas) |d| c.mj_deleteData(d);

        // 100 control sequences: sine waves with a random phase per actuator
        const duration = 3.0;
        const nbatch = 100;
        const nu: usize = @intCast(humanoid_model.*.nu);
        const nstep: usize = @intFromFloat(duration / humanoid_model.*.opt.timestep);
        var prng: std.Random.DefaultPrng = .init(1);
        const random = prng.random();
        const phase = try arena.alloc(f64, nbatch * nu);
        for (phase) |*p| p.* = 2 * std.math.pi * random.float(f64);
        const control = try arena.alloc(f64, nbatch * nstep * nu);
        for (0..nbatch) |b| {
            for (0..nstep) |t| {
                // times = linspace(0, duration, nstep)
                const time = duration * @as(f64, @floatFromInt(t)) / @as(f64, @floatFromInt(nstep - 1));
                for (0..nu) |u| control[(b * nstep + t) * nu + u] = @sin(2 * std.math.pi * time + phase[b * nu + u]);
            }
        }

        // jumping humanoids on a 10 x 10 grid (state index 0 is time, qpos x and y at 1 and 2)
        humanoid_data.*.qvel[2] = 4;
        const initial_states = try tiledState(arena, humanoid_model, humanoid_data, nbatch);
        const nstate = initial_states.len / nbatch;
        xyGrid(initial_states, nbatch, nstate, 10, 1.0);

        const state = try arena.alloc(f64, nbatch * nstep * nstate);
        const start = std.Io.Timestamp.now(io, .awake);
        try rollout(&.{humanoid_model}, humanoid_datas, nstep, initial_states, .{ .control = control, .state = state });
        std.log.info("humanoids: rollout time {d:.2} s", .{seconds(io, start)});

        // render at 120 Hz, play at half speed
        var cam: c.mjvCamera = undefined;
        c.mjv_defaultCamera(&cam);
        cam.distance = 10;
        cam.azimuth = 45;
        cam.elevation = -15;
        cam.lookat = .{ 0, 0, 0 };
        humanoid_model.*.vis.global.fovy = 60;
        try renderMany(io, gpa, &.{humanoid_model}, humanoid_data, state, nbatch, nstep, 120, &cam, null, .{ 0, 0, 10 }, 480, 640, 60, "rollout_humanoids.mp4");
    }

    //------------------------------------ benchmark ------------------------------------

    {
        // 1000 tops for 100 steps, on the calling thread vs all threads
        const nbatch = 1000;
        const nstep = 100;
        c.mj_resetDataKeyframe(top_model, top_data, 0);
        const initial_states = try tiledState(arena, top_model, top_data, nbatch);
        const nstate = initial_states.len / nbatch;
        const state = try arena.alloc(f64, nbatch * nstep * nstate);

        for ([_][]const [*c]c.mjData{ top_datas[0..1], top_datas }) |datas| {
            const start = std.Io.Timestamp.now(io, .awake);
            try rollout(&.{top_model}, datas, nstep, initial_states, .{ .state = state });
            const t = seconds(io, start);
            std.log.info("benchmark: {d} thread(s): {d:.0} steps per second", .{ datas.len, nbatch * nstep / t });
        }
    }
}

/// one scratch mjData per thread
fn makeDatas(arena: std.mem.Allocator, m: [*c]const c.mjModel, n: usize) ![]const [*c]c.mjData {
    const datas = try arena.alloc([*c]c.mjData, n);
    for (datas) |*d| d.* = c.mj_makeData(m);
    return datas;
}

/// full physics state of d, repeated nbatch times (get_state in the notebook)
fn tiledState(arena: std.mem.Allocator, m: [*c]const c.mjModel, d: [*c]c.mjData, nbatch: usize) ![]f64 {
    const nstate: usize = @intCast(c.mj_stateSize(m, c.mjSTATE_FULLPHYSICS));
    const states = try arena.alloc(f64, nbatch * nstate);
    c.mj_getState(m, d, states.ptr, c.mjSTATE_FULLPHYSICS);
    for (1..nbatch) |b| @memcpy(states[b * nstate ..][0..nstate], states[0..nstate]);
    return states;
}

/// place the rollouts on an x-y grid (xy_grid in the notebook); qpos x, y are state[1..3]
fn xyGrid(states: []f64, nbatch: usize, nstate: usize, ncols: usize, spacing: f64) void {
    const nrows = nbatch / ncols;
    const xmax = @as(f64, @floatFromInt(nrows - 1)) * spacing / 2;
    const ymax = @as(f64, @floatFromInt(ncols - 1)) * spacing / 2;
    for (0..nbatch) |b| {
        // meshgrid(rows, cols) flattened: b = col * nrows + row
        const row = b % nrows;
        const col = b / nrows;
        states[b * nstate + 1] = -xmax + spacing * @as(f64, @floatFromInt(row));
        states[b * nstate + 2] = -ymax + spacing * @as(f64, @floatFromInt(col));
    }
}

/// Render all rollouts into one video (render_many in the notebook): the first
/// rollout fills the scene, the others are added with mjv_addGeoms.
fn renderMany(
    io: std.Io,
    gpa: std.mem.Allocator,
    models: []const [*c]c.mjModel,
    d: [*c]c.mjData,
    state: []const f64,
    nbatch: usize,
    nstep: usize,
    framerate: f64,
    camera: ?*c.mjvCamera,
    option: ?*const c.mjvOption,
    light_pos: ?[3]f32,
    height: usize,
    width: usize,
    video_fps: f64,
    path: []const u8,
) !void {
    const m0 = models[0];
    const nstate: usize = @intCast(c.mj_stateSize(m0, c.mjSTATE_FULLPHYSICS));
    var opt: c.mjvOption = undefined;
    c.mjv_defaultOption(&opt);
    const vopt = option orelse &opt;
    var pert: c.mjvPerturb = undefined;
    c.mjv_defaultPerturb(&pert);

    var renderer = try Renderer.init(gpa, m0, height, width);
    defer renderer.deinit();
    var video = try media.VideoWriter.init(io, path, width, height, video_fps);
    errdefer video.finish() catch {};

    const start = std.Io.Timestamp.now(io, .awake);
    var nframe: usize = 0;
    for (0..nstep) |i| {
        if (@as(f64, @floatFromInt(nframe)) >= @as(f64, @floatFromInt(i)) * m0.*.opt.timestep * framerate) continue;
        for (0..nbatch) |j| {
            const m = if (models.len == 1) m0 else models[j];
            c.mj_setState(m, d, state[(j * nstep + i) * nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
            c.mj_forward(m, d);
            if (j == 0) {
                try renderer.updateScene(d, if (camera) |cam| .{ .custom = cam } else .free, vopt);
            } else {
                c.mjv_addGeoms(m, d, vopt, &pert, c.mjCAT_DYNAMIC, &renderer.scene);
            }
        }

        // add a spot light, if requested
        if (light_pos) |pos| {
            const scn = &renderer.scene;
            scn.lights[@intCast(scn.nlight)] = .{
                .pos = pos,
                .dir = .{ 0, 0, -1 },
                .type = c.mjLIGHT_SPOT,
                .attenuation = .{ 1, 0, 0 },
                .cutoff = 45,
                .exponent = 10,
                .ambient = .{ 0, 0, 0 },
                .diffuse = .{ 0.8, 0.8, 0.8 },
                .specular = .{ 0.3, 0.3, 0.3 },
                .headlight = 0,
                .castshadow = 1,
            };
            scn.nlight += 1;
        }

        try video.write(renderer.render());
        nframe += 1;
    }
    try video.finish();
    std.log.info("wrote {s} ({d} frames), render time {d:.2} s", .{ path, nframe, seconds(io, start) });
}

fn seconds(io: std.Io, start: std.Io.Timestamp) f64 {
    const elapsed = start.durationTo(std.Io.Timestamp.now(io, .awake));
    return @as(f64, @floatFromInt(elapsed.nanoseconds)) / std.time.ns_per_s;
}
