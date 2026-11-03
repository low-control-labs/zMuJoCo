const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
const c = lib_mujoco.c;
const Lqr = @import("lqr").Lqr;
const Renderer = @import("renderer").Renderer;
const media = @import("media");

// LQR balancing of the humanoid on one leg, a port of python/LQR.ipynb.
// Computes the setpoint and the LQR gain (zig/lqr.zig), simulates the closed
// loop and records it to a video (needs ffmpeg) plus an image of the setpoint.
// USAGE: zlqr [--noise] [--duration SECONDS] [--out FILE.mp4] [modelfile]

// video
const width = 1280;
const height = 720;
const framerate = 60;
const total_rotation = 15; // camera orbit, degrees

// perturbation
const ctrl_rate: f64 = 0.8; // seconds
const balance_std: f64 = 0.01; // actuator units
const other_std: f64 = 0.08; // actuator units

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const gpa = init.gpa;
    const arena = init.arena.allocator();

    // command-line arguments
    const args = try init.minimal.args.toSlice(arena);
    var noise = false;
    var duration: ?f64 = null;
    var out_path: []const u8 = "lqr.mp4";
    var model_path: [*:0]const u8 = "model/humanoid/humanoid.xml";
    var a: usize = 1;
    while (a < args.len) : (a += 1) {
        if (std.mem.eql(u8, args[a], "--noise")) {
            noise = true;
        } else if (std.mem.eql(u8, args[a], "--duration") and a + 1 < args.len) {
            a += 1;
            duration = try std.fmt.parseFloat(f64, args[a]);
        } else if (std.mem.eql(u8, args[a], "--out") and a + 1 < args.len) {
            a += 1;
            out_path = args[a];
        } else {
            model_path = args[a].ptr;
        }
    }
    const sim_duration = duration orelse if (noise) @as(f64, 12) else 5;

    // load model, make data
    var load_error: [1000]u8 = undefined;
    const m = c.mj_loadXML(model_path, null, &load_error, load_error.len);
    if (m == null) {
        std.log.err("could not load model: {s}", .{std.mem.sliceTo(&load_error, 0)});
        return error.ModelLoadFailed;
    }
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    // setpoint and LQR gain
    var lqr = try Lqr.init(arena, m, d);
    const nu = lqr.nu;

    //-------------------------------------- rendering ------------------------------------------

    var renderer = try Renderer.init(gpa, m, height, width);
    defer renderer.deinit();

    // image of the setpoint pose
    lqr.setPose(m, d);
    try renderer.updateScene(d, .free, null);
    try media.writeImage(io, gpa, "lqr_setpoint.png", renderer.render(), width, height);
    std.log.info("wrote lqr_setpoint.png", .{});

    // camera orbiting the humanoid, contact forces visualized at 1cm/N
    var camera: c.mjvCamera = undefined;
    c.mjv_defaultFreeCamera(m, &camera);
    camera.distance = 2.3;
    var option: c.mjvOption = undefined;
    c.mjv_defaultOption(&option);
    option.flags[c.mjVIS_CONTACTFORCE] = 1;
    m.*.vis.map.force = 0.01;

    //------------------------------------- perturbation ----------------------------------------

    const timestep = m.*.opt.timestep;
    const nsteps: usize = @intFromFloat(@ceil(sim_duration / timestep));
    // only allocated and used with --noise
    var perturb: []f64 = undefined;
    var ctrl_std: []f64 = undefined;
    if (noise) {
        // white noise, smoothed with a normalized gaussian kernel (np.convolve mode 'same')
        var prng: std.Random.DefaultPrng = .init(1);
        const random = prng.random();
        const white = try arena.alloc(f64, nsteps * nu);
        for (white) |*x| x.* = random.floatNorm(f64);

        const kwidth: usize = @max(1, @as(usize, @intFromFloat(@as(f64, @floatFromInt(nsteps)) * ctrl_rate / sim_duration)));
        const kernel = try arena.alloc(f64, kwidth);
        var knorm: f64 = 0;
        for (kernel, 0..) |*k, i| {
            const x = if (kwidth == 1) 0 else -3 + 6 * @as(f64, @floatFromInt(i)) / @as(f64, @floatFromInt(kwidth - 1));
            k.* = @exp(-0.5 * x * x);
            knorm += k.* * k.*;
        }
        for (kernel) |*k| k.* /= @sqrt(knorm);

        perturb = try arena.alloc(f64, nsteps * nu);
        const center = (kwidth - 1) / 2;
        for (0..nsteps) |t| {
            for (0..nu) |u| {
                var sum: f64 = 0;
                for (0..kwidth) |k| {
                    // full-convolution index t + center = j + k
                    const j = @as(isize, @intCast(t + center)) - @as(isize, @intCast(k));
                    if (j >= 0 and j < @as(isize, @intCast(nsteps))) sum += white[@as(usize, @intCast(j)) * nu + u] * kernel[k];
                }
                perturb[t * nu + u] = sum;
            }
        }

        // different noise scale for balance and other actuators
        ctrl_std = try arena.alloc(f64, nu);
        for (0..nu) |u| ctrl_std[u] = if (lqr.isBalanceActuator(m, u)) balance_std else other_std;
    }

    //-------------------------------------- simulation -----------------------------------------

    var video = try media.VideoWriter.init(io, out_path, width, height, framerate);
    errdefer video.finish() catch {};

    lqr.setPose(m, d);
    var nframe: usize = 0;
    var step: usize = 0;
    while (d.*.time < sim_duration) : (step += 1) {
        // LQR control law, plus optional perturbation
        lqr.control(m, d);
        if (noise and step < nsteps) {
            for (0..nu) |u| d.*.ctrl[u] += ctrl_std[u] * perturb[step * nu + u];
        }

        c.mj_step(m, d);

        // render a frame when due
        if (@as(f64, @floatFromInt(nframe)) < d.*.time * framerate) {
            const t = d.*.time / sim_duration;
            camera.azimuth = 100 + (1 - @cos(t * 2 * std.math.pi)) * total_rotation;
            try renderer.updateScene(d, .{ .custom = &camera }, &option);
            try video.write(renderer.render());
            nframe += 1;
        }
    }
    try video.finish();
    std.log.info("wrote {s} ({d} frames, {d:.1}s simulated)", .{ out_path, nframe, d.*.time });
}
