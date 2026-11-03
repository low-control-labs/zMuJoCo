const std = @import("std");
const zsimulate = @import("zsimulate");
const c = zsimulate.c;
const Lqr = @import("lqr").Lqr;

// LQR balancing of the humanoid on one leg (python/LQR.ipynb) in the simulate
// viewer, using the passive viewer: this program owns m/d and steps them in
// real time, the viewer only renders. Push the humanoid with ctrl + right drag;
// Backspace resets to the balance pose.
// USAGE: zlqr_viewer [--noise] [modelfile]

// perturbation, as in the notebook: white noise smoothed with a normalized
// gaussian kernel ctrl_rate seconds wide, generated here as a stream
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
    var model_path: [*:0]const u8 = "model/humanoid/humanoid.xml";
    for (args[1..]) |arg| {
        if (std.mem.eql(u8, arg, "--noise")) {
            noise = true;
        } else {
            model_path = arg.ptr;
        }
    }

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

    // setpoint and LQR gain, start in the balance pose
    var lqr = try Lqr.init(arena, m, d);
    lqr.setPose(m, d);

    // noise scale per actuator
    const nu = lqr.nu;
    const ctrl_std = try arena.alloc(f64, nu);
    for (0..nu) |u| ctrl_std[u] = if (lqr.isBalanceActuator(m, u)) balance_std else other_std;

    // smoothing kernel: exp(-x^2/2) on linspace(-3, 3, width), normalized (L2)
    const timestep = m.*.opt.timestep;
    const kwidth: usize = @max(1, @as(usize, @intFromFloat(ctrl_rate / timestep)));
    const kernel = try arena.alloc(f64, kwidth);
    var knorm: f64 = 0;
    for (kernel, 0..) |*k, i| {
        const x = if (kwidth == 1) 0 else -3 + 6 * @as(f64, @floatFromInt(i)) / @as(f64, @floatFromInt(kwidth - 1));
        k.* = @exp(-0.5 * x * x);
        knorm += k.* * k.*;
    }
    for (kernel) |*k| k.* /= @sqrt(knorm);

    // ring buffer of the last kwidth white-noise samples (kwidth x nu); starting
    // from zeros gives the notebook's ramp-in at the start of the signal
    const white = try arena.alloc(f64, kwidth * nu);
    @memset(white, 0);
    var head: usize = 0;
    var prng: std.Random.DefaultPrng = .init(1);
    const random = prng.random();

    // contact forces at 1cm/N like the notebook's video; must be set before the
    // launch, the passive viewer copies mjModel.vis only when it loads the model
    m.*.vis.map.force = 0.01;

    const viewer = try zsimulate.launchPassive(gpa, m, d, .{});
    defer viewer.deinit();

    // fit the free camera to the model, as with viewer.cam in Python, and show
    // the contact forces
    viewer.lock();
    c.mjv_defaultFreeCamera(m, viewer.cam());
    viewer.opt().flags[c.mjVIS_CONTACTFORCE] = 1;
    viewer.unlock();

    var last_time = d.*.time;
    while (viewer.isRunning()) {
        const step_start = std.Io.Timestamp.now(io, .awake);

        // the viewer's reset (Backspace) happens in sync(): back to the balance pose
        if (d.*.time < last_time) {
            lqr.setPose(m, d);
            @memset(white, 0);
        }

        // LQR control law, plus optional perturbation
        lqr.control(m, d);
        if (noise) {
            // new white sample, then the kernel over the last kwidth samples
            for (0..nu) |u| white[head * nu + u] = random.floatNorm(f64);
            head = (head + 1) % kwidth;
            for (0..nu) |u| {
                var sum: f64 = 0;
                for (0..kwidth) |k| sum += kernel[k] * white[((head + k) % kwidth) * nu + u];
                d.*.ctrl[u] += ctrl_std[u] * sum;
            }
        }

        c.mj_step(m, d);
        last_time = d.*.time;

        // update the viewer, then sleep for the rest of the timestep (real time)
        viewer.sync(false);
        const elapsed = step_start.durationTo(std.Io.Timestamp.now(io, .awake));
        const remaining: i96 = @as(i96, @intFromFloat(timestep * std.time.ns_per_s)) - elapsed.nanoseconds;
        if (remaining > 0) io.sleep(.fromNanoseconds(remaining), .awake) catch {};
    }
}
