const std = @import("std");
const zsimulate = @import("zsimulate");
const c = zsimulate.c;

// Simulate viewer with an optional controller and logging, built on zsimulate.
// USAGE: viewer [--sine AMP HZ] [--log SECONDS] [modelfile]

// settings read by the controller (set once in main, before the viewer starts)
var sine_amp: f64 = 0;
var sine_hz: f64 = 0;
var log_period: f64 = 0;
var next_log: f64 = 0;

// runs inside simulate's physics loop, under its lock
fn control(m: [*c]const c.mjModel, d: [*c]c.mjData) callconv(.c) void {
    const t = d.*.time;

    // drive every actuator with a sine wave, clamped to its ctrlrange
    if (sine_amp != 0) {
        const u = sine_amp * @sin(2 * std.math.pi * sine_hz * t);
        for (0..@intCast(m.*.nu)) |i| {
            d.*.ctrl[i] = if (m.*.actuator_ctrllimited[i])
                std.math.clamp(u, m.*.actuator_ctrlrange[2 * i], m.*.actuator_ctrlrange[2 * i + 1])
            else
                u;
        }
    }

    // periodic state log, in simulation time
    if (log_period > 0) {
        if (t < next_log - log_period) next_log = t; // model reset or reloaded
        if (t >= next_log) {
            next_log = t + log_period;
            c.mj_energyPos(m, d);
            c.mj_energyVel(m, d);
            std.log.info("t={d:.3} ncon={d} energy={d:.4} (pot {d:.4}, kin {d:.4})", .{
                t, d.*.ncon, d.*.energy[0] + d.*.energy[1], d.*.energy[0], d.*.energy[1],
            });
        }
    }
}

fn warning(msg: [*c]const u8) callconv(.c) void {
    std.log.warn("mujoco: {s}", .{std.mem.span(msg)});
}

fn fatal(msg: [*c]const u8) callconv(.c) void {
    std.log.err("mujoco: {s}", .{std.mem.span(msg)});
    std.process.exit(1);
}

fn parseArg(args: []const [:0]const u8, i: usize) !f64 {
    if (i >= args.len) {
        std.log.err("missing value for {s}", .{args[i - 1]});
        return error.InvalidArguments;
    }
    return std.fmt.parseFloat(f64, args[i]);
}

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const args = try init.minimal.args.toSlice(init.arena.allocator());

    var model: ?[]const u8 = null;
    var i: usize = 1;
    while (i < args.len) : (i += 1) {
        if (std.mem.eql(u8, args[i], "--sine")) {
            sine_amp = try parseArg(args, i + 1);
            sine_hz = try parseArg(args, i + 2);
            i += 2;
        } else if (std.mem.eql(u8, args[i], "--log")) {
            log_period = try parseArg(args, i + 1);
            i += 1;
        } else {
            model = args[i];
        }
    }

    zsimulate.setMessageHandlers(warning, fatal);
    if (sine_amp != 0 or log_period > 0) zsimulate.setController(control);

    try zsimulate.launchFromPath(io, model, .{});
}
