const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
pub const c = lib_mujoco.c;
const build_options = @import("build_options");

// The simulate viewer, a port of python/mujoco/viewer.py on top of the C layer
// in zig/mujoco/simulate.cc. The simulate/ sources are used unmodified.
//   launch / launchFromPath: simulate runs the physics (blocking)
//   launchPassive:           you own m/d and step them, the viewer only renders

//------------------------------------ C layer --------------------------------------------------

const Sim = opaque {};
pub const KeyCallback = *const fn (key: c_int, userdata: ?*anyopaque) callconv(.c) void;

pub const Text = extern struct {
    font: c_int = c.mjFONT_NORMAL,
    gridpos: c_int = c.mjGRID_TOPLEFT,
    text1: [*:0]const u8,
    text2: [*:0]const u8 = "",
};
pub const Figure = extern struct { viewport: c.mjrRect, figure: *const c.mjvFigure };
pub const Image = extern struct { viewport: c.mjrRect, rgb: [*]const u8, width: c_int, height: c_int };

extern fn mjsim_create(user_scn: ?*c.mjvScene, is_passive: c_int, key_callback: ?KeyCallback, userdata: ?*anyopaque) *Sim;
extern fn mjsim_destroy(h: *Sim) void;
extern fn mjsim_free(h: *Sim) void;
extern fn mjsim_renderLoop(h: *Sim) void;
extern fn mjsim_exit(h: *Sim) void;
extern fn mjsim_loadMessage(h: *Sim, displayed_filename: [*:0]const u8) void;
extern fn mjsim_loadMessageClear(h: *Sim) void;
extern fn mjsim_load(h: *Sim, m: *c.mjModel, d: *c.mjData, displayed_filename: [*:0]const u8) void;
extern fn mjsim_sync(h: *Sim, state_only: c_int) void;
extern fn mjsim_lock(h: *Sim) void;
extern fn mjsim_unlock(h: *Sim) void;
extern fn mjsim_addToHistory(h: *Sim) void;
extern fn mjsim_cam(h: *Sim) *c.mjvCamera;
extern fn mjsim_opt(h: *Sim) *c.mjvOption;
extern fn mjsim_pert(h: *Sim) *c.mjvPerturb;
extern fn mjsim_viewport(h: *Sim) c.mjrRect;
extern fn mjsim_setFigures(h: *Sim, figures: [*]const Figure, n: c_int) void;
extern fn mjsim_clearFigures(h: *Sim) void;
extern fn mjsim_setTexts(h: *Sim, texts: [*]const Text, n: c_int) void;
extern fn mjsim_clearTexts(h: *Sim) void;
extern fn mjsim_setImages(h: *Sim, images: [*]const Image, n: c_int) void;
extern fn mjsim_clearImages(h: *Sim) void;
extern fn mjsim_updateHField(h: *Sim, hfieldid: c_int) void;
extern fn mjsim_updateMesh(h: *Sim, meshid: c_int) void;
extern fn mjsim_updateTexture(h: *Sim, texid: c_int) void;
extern fn mjsim_exitrequest(h: *Sim) c_int;
extern fn mjsim_run(h: *Sim) c_int;
extern fn mjsim_uiloadrequest(h: *Sim) c_int;
extern fn mjsim_uiloadrequestDecrement(h: *Sim) void;
extern fn mjsim_droploadrequest(h: *Sim) c_int;
extern fn mjsim_setDroploadrequest(h: *Sim, droploadrequest: c_int) void;
extern fn mjsim_dropfilename(h: *Sim) [*:0]const u8;
extern fn mjsim_setLoadError(h: *Sim, err: [*:0]const u8) void;
extern fn mjsim_ctrlNoiseStd(h: *Sim) f64;
extern fn mjsim_ctrlNoiseRate(h: *Sim) f64;
extern fn mjsim_realTimeIndex(h: *Sim) c_int;
extern fn mjsim_percentRealTime(index: c_int) f32;
extern fn mjsim_speedChanged(h: *Sim) c_int;
extern fn mjsim_setSpeedChanged(h: *Sim, speed_changed: c_int) void;
extern fn mjsim_setMeasuredSlowdown(h: *Sim, measured_slowdown: f32) void;
extern fn mjsim_refreshRate(h: *Sim) c_int;
extern fn mjsim_busywait(h: *Sim) c_int;
extern fn mjsim_setUi0Enable(h: *Sim, enabled: c_int) void;
extern fn mjsim_setUi1Enable(h: *Sim, enabled: c_int) void;
extern fn mjsim_setColor(h: *Sim, color: c_int) void;

//------------------------------------ options --------------------------------------------------

pub const Color = enum(c_int) { default, orange, white, black };

pub const Options = struct {
    show_left_ui: bool = true,
    show_right_ui: bool = true,
    /// UI color scheme; default from -Dui-color
    color: Color = @enumFromInt(build_options.ui_color),
    /// called with the GLFW key code on key-down events (passive mode)
    key_callback: ?KeyCallback = null,
    key_userdata: ?*anyopaque = null,
};

// constants from viewer.py
const max_sync_misalign = 0.1; // maximum mis-alignment before re-sync (simulation seconds)
const sim_refresh_fraction = 0.7; // fraction of refresh available for simulation
const max_geom = 100000; // Simulate::kMaxGeom

fn create(user_scn: ?*c.mjvScene, is_passive: bool, options: Options) *Sim {
    const sim = mjsim_create(user_scn, @intFromBool(is_passive), options.key_callback, options.key_userdata);
    mjsim_setUi0Enable(sim, @intFromBool(options.show_left_ui));
    mjsim_setUi1Enable(sim, @intFromBool(options.show_right_ui));
    mjsim_setColor(sim, @intFromEnum(options.color));
    return sim;
}

/// Install (or with null, remove) a controller, called every physics step
/// through mjcb_control.
pub fn setController(controller: c.mjfGeneric) void {
    c.mjcb_control = controller;
}

/// Route MuJoCo warnings and errors to handlers. An error handler should not return.
pub fn setMessageHandlers(
    warning: ?*const fn ([*c]const u8) callconv(.c) void,
    err: ?*const fn ([*c]const u8) callconv(.c) void,
) void {
    c.mju_user_warning = warning;
    c.mju_user_error = err;
}

//-------------------------------------- launch -------------------------------------------------

/// Launch the viewer with simulate running the physics; blocks until the
/// window is closed. m/d are owned by the caller; with null, a model can be
/// dragged into the window.
pub fn launch(io: std.Io, m: ?*c.mjModel, d: ?*c.mjData, options: Options) !void {
    var loader: Loader = if (m) |model| .{ .given = .{ .m = model, .d = d } } else .none;
    try launchInternal(io, &loader, options);
}

/// Launch the viewer on a model file, or empty (drag a model in) with null;
/// blocks until the window is closed.
pub fn launchFromPath(io: std.Io, path: ?[]const u8, options: Options) !void {
    var loader: Loader = .none;
    if (path) |p| {
        loader = .{ .file = .{} };
        try loader.file.setPath(p);
    }
    try launchInternal(io, &loader, options);
}

fn launchInternal(io: std.Io, loader: *Loader, options: Options) !void {
    const sim = create(null, false, options);
    errdefer mjsim_free(sim);
    const physics = try std.Thread.spawn(.{}, physicsLoop, .{ io, sim, loader });
    mjsim_renderLoop(sim);
    physics.join();

    // free the viewer before the models it showed
    mjsim_free(sim);
    loader.deinit();
}

// where models come from: given by the caller, or loaded from a file (own m/d)
const Loader = union(enum) {
    none,
    given: struct { m: *c.mjModel, d: ?*c.mjData, own_d: bool = false },
    file: File,

    const File = struct {
        path: [1000:0]u8 = std.mem.zeroes([1000:0]u8),
        m: ?*c.mjModel = null,
        d: ?*c.mjData = null,

        fn setPath(f: *File, path: []const u8) !void {
            if (path.len >= f.path.len) return error.PathTooLong;
            @memcpy(f.path[0..path.len], path);
            f.path[path.len] = 0;
        }
    };

    fn deinit(loader: *Loader) void {
        switch (loader.*) {
            .none => {},
            .given => |g| if (g.own_d) c.mj_deleteData(g.d),
            .file => |f| {
                if (f.d) |d| c.mj_deleteData(d);
                if (f.m) |m| c.mj_deleteModel(m);
            },
        }
    }
};

// _reload in viewer.py: load the model and hand it to the viewer
fn reload(sim: *Sim, loader: *Loader) ?struct { m: *c.mjModel, d: *c.mjData } {
    switch (loader.*) {
        .none => return null,
        .given => |*g| {
            mjsim_loadMessage(sim, "");
            if (g.d == null) {
                g.d = c.mj_makeData(g.m);
                g.own_d = true;
            }
            mjsim_load(sim, g.m, g.d.?, "");
            mjsim_setLoadError(sim, "");
            return .{ .m = g.m, .d = g.d.? };
        },
        .file => |*f| {
            const path: [*:0]const u8 = &f.path;
            mjsim_loadMessage(sim, path);
            var load_error = std.mem.zeroes([1000:0]u8);
            const span = std.mem.sliceTo(path, 0);
            const m = if (std.mem.endsWith(u8, span, ".mjb"))
                c.mj_loadModel(path, null)
            else
                c.mj_loadXML(path, null, &load_error, load_error.len);
            if (m == null) {
                const message: [*:0]const u8 = if (load_error[0] != 0) &load_error else "could not load model";
                mjsim_setLoadError(sim, message);
                mjsim_loadMessageClear(sim);
                return null;
            }
            const d = c.mj_makeData(m);
            mjsim_load(sim, m, d, path);
            mjsim_setLoadError(sim, "");

            // the viewer has taken over the new model, free the previous one
            if (f.d) |old| c.mj_deleteData(old);
            if (f.m) |old| c.mj_deleteModel(old);
            f.m = m;
            f.d = d;
            return .{ .m = m, .d = d };
        },
    }
}

// _physics_loop in viewer.py: simulate's physics loop, in a separate thread
fn physicsLoop(io: std.Io, sim: *Sim, loader: *Loader) void {
    var m: ?*c.mjModel = null;
    var d: ?*c.mjData = null;
    var ctrl_noise: [*c]f64 = null;
    defer if (ctrl_noise != null) c.mju_free(ctrl_noise);
    var do_reload = true;

    // CPU-sim synchronization point
    var synccpu: f64 = 0;
    var syncsim: f64 = 0;

    // run until asked to exit
    while (mjsim_exitrequest(sim) == 0) {
        if (mjsim_droploadrequest(sim) != 0) {
            mjsim_setDroploadrequest(sim, 0);
            if (loader.* != .file) {
                loader.deinit();
                loader.* = .{ .file = .{} };
            }
            loader.file.setPath(std.mem.span(mjsim_dropfilename(sim))) catch {};
            do_reload = true;
        }
        if (mjsim_uiloadrequest(sim) != 0) {
            mjsim_uiloadrequestDecrement(sim);
            do_reload = true;
        }
        if (do_reload) {
            if (reload(sim, loader)) |loaded| {
                m = loaded.m;
                d = loaded.d;
                if (ctrl_noise != null) c.mju_free(ctrl_noise);
                const nu: usize = @intCast(loaded.m.nu);
                ctrl_noise = @ptrCast(@alignCast(c.mju_malloc(@max(1, nu) * @sizeOf(f64))));
                @memset(ctrl_noise[0..nu], 0);
            }
        }
        do_reload = false;

        // sleep for 1 ms or yield, to let the main thread run
        if (mjsim_run(sim) != 0 and mjsim_busywait(sim) != 0) {
            std.Thread.yield() catch {};
        } else {
            io.sleep(.fromMilliseconds(1), .awake) catch {};
        }

        mjsim_lock(sim);
        defer mjsim_unlock(sim);
        const model = m orelse continue;
        const data = d.?;
        if (mjsim_run(sim) != 0) {
            var stepped = false;
            // CPU time at the start of the iteration
            const startcpu = now(io);
            const elapsedcpu = startcpu - synccpu;
            const elapsedsim = data.time - syncsim;

            // inject noise, converting rate and scale to discrete time (Ornstein-Uhlenbeck)
            const noise_std = mjsim_ctrlNoiseStd(sim);
            if (noise_std != 0) {
                const rate = @exp(-model.opt.timestep / @max(mjsim_ctrlNoiseRate(sim), c.mjMINVAL));
                const scale = noise_std * @sqrt(1 - rate * rate);
                for (0..@intCast(model.nu)) |i| {
                    ctrl_noise[i] = rate * ctrl_noise[i] + scale * c.mju_standardNormal(null);
                    data.ctrl[i] = ctrl_noise[i];
                }
            }

            // requested slow-down factor
            const slowdown = 100 / @as(f64, mjsim_percentRealTime(mjsim_realTimeIndex(sim)));

            // misalignment: distance from target sim time > max_sync_misalign
            const misaligned = @abs(elapsedcpu / slowdown - elapsedsim) > max_sync_misalign;

            if (elapsedsim < 0 or elapsedcpu < 0 or synccpu == 0 or misaligned or mjsim_speedChanged(sim) != 0) {
                // out of sync (for any reason): re-sync, run a single step
                synccpu = startcpu;
                syncsim = data.time;
                mjsim_setSpeedChanged(sim, 0);
                c.mj_step(model, data);
                stepped = true;
            } else {
                // in sync: step until ahead of cpu
                var measured = false;
                const prevsim = data.time;
                const refreshtime = sim_refresh_fraction / @as(f64, @floatFromInt(mjsim_refreshRate(sim)));
                while ((data.time - syncsim) * slowdown < now(io) - synccpu and now(io) - startcpu < refreshtime) {
                    // measure slowdown before the first step
                    if (!measured and elapsedsim != 0) {
                        mjsim_setMeasuredSlowdown(sim, @floatCast(elapsedcpu / elapsedsim));
                        measured = true;
                    }
                    c.mj_step(model, data);
                    stepped = true;
                    if (data.time < prevsim) break; // reset
                }
            }

            // save the current state to the history buffer
            if (stepped) mjsim_addToHistory(sim);
        } else {
            // paused: mj_forward to update rendering and joint sliders
            c.mj_forward(model, data);
            mjsim_setSpeedChanged(sim, 1);
        }
    }
}

fn now(io: std.Io) f64 {
    const t = std.Io.Timestamp.now(io, .awake);
    return @as(f64, @floatFromInt(t.nanoseconds)) / std.time.ns_per_s;
}

//---------------------------------- passive viewer ---------------------------------------------

/// Launch a passive viewer, without blocking: the caller owns m/d and steps
/// them, and calls sync() to update the viewer. Returns a handle; deinit()
/// closes the viewer.
pub fn launchPassive(gpa: std.mem.Allocator, m: *c.mjModel, d: *c.mjData, options: Options) !*Handle {
    c.mj_forward(m, d);

    const handle = try gpa.create(Handle);
    errdefer gpa.destroy(handle);
    handle.* = .{ .gpa = gpa };
    c.mjv_defaultScene(&handle.user_scn);
    c.mjv_makeScene(m, &handle.user_scn, max_geom);

    // the viewer and its window are created on the render thread
    handle.thread = try std.Thread.spawn(.{}, Handle.renderThread, .{ handle, options });
    while (handle.created.load(.acquire) == 0) std.Thread.yield() catch {};

    // hand over the model (blocks until the render thread has loaded it)
    const sim = handle.sim.?;
    mjsim_loadMessage(sim, "");
    mjsim_load(sim, m, d, "");
    mjsim_setLoadError(sim, "");
    return handle;
}

pub const Handle = struct {
    gpa: std.mem.Allocator,
    sim: ?*Sim = null,
    user_scn: c.mjvScene = undefined,
    thread: std.Thread = undefined,
    created: std.atomic.Value(u32) = .init(0),
    render_done: std.atomic.Value(u32) = .init(0),
    release: std.atomic.Value(u32) = .init(0),

    fn renderThread(h: *Handle, options: Options) void {
        const sim = create(&h.user_scn, true, options);
        h.sim = sim;
        h.created.store(1, .release);
        mjsim_renderLoop(sim);
        h.render_done.store(1, .release);

        // keep the viewer alive until the handle is released, so that calls
        // from the user's thread stay valid after the window is closed
        while (h.release.load(.acquire) == 0) std.Thread.yield() catch {};
        mjsim_free(sim);
    }

    /// Whether the window is still open.
    pub fn isRunning(h: *Handle) bool {
        return h.render_done.load(.acquire) == 0;
    }

    /// Exchange state with the viewer and update the visualization. With
    /// state_only, only the integration state of d is synced.
    pub fn sync(h: *Handle, state_only: bool) void {
        if (h.isRunning()) mjsim_sync(h.sim.?, @intFromBool(state_only));
    }

    /// Lock the viewer while modifying m/d outside of sync.
    pub fn lock(h: *Handle) void {
        mjsim_lock(h.sim.?);
    }

    pub fn unlock(h: *Handle) void {
        mjsim_unlock(h.sim.?);
    }

    pub fn cam(h: *Handle) *c.mjvCamera {
        return mjsim_cam(h.sim.?);
    }

    pub fn opt(h: *Handle) *c.mjvOption {
        return mjsim_opt(h.sim.?);
    }

    pub fn perturb(h: *Handle) *c.mjvPerturb {
        return mjsim_pert(h.sim.?);
    }

    /// Extra geoms drawn by the viewer, added by the user (scn.ngeom, scn.geoms).
    pub fn userScn(h: *Handle) *c.mjvScene {
        return &h.user_scn;
    }

    pub fn viewport(h: *Handle) c.mjrRect {
        return mjsim_viewport(h.sim.?);
    }

    pub fn setTexts(h: *Handle, texts: []const Text) void {
        if (h.isRunning()) mjsim_setTexts(h.sim.?, texts.ptr, @intCast(texts.len));
    }

    pub fn clearTexts(h: *Handle) void {
        if (h.isRunning()) mjsim_clearTexts(h.sim.?);
    }

    pub fn setFigures(h: *Handle, figures: []const Figure) void {
        if (h.isRunning()) mjsim_setFigures(h.sim.?, figures.ptr, @intCast(figures.len));
    }

    pub fn clearFigures(h: *Handle) void {
        if (h.isRunning()) mjsim_clearFigures(h.sim.?);
    }

    pub fn setImages(h: *Handle, images: []const Image) void {
        if (h.isRunning()) mjsim_setImages(h.sim.?, images.ptr, @intCast(images.len));
    }

    pub fn clearImages(h: *Handle) void {
        if (h.isRunning()) mjsim_clearImages(h.sim.?);
    }

    pub fn updateHField(h: *Handle, hfieldid: c_int) void {
        if (h.isRunning()) mjsim_updateHField(h.sim.?, hfieldid);
    }

    pub fn updateMesh(h: *Handle, meshid: c_int) void {
        if (h.isRunning()) mjsim_updateMesh(h.sim.?, meshid);
    }

    pub fn updateTexture(h: *Handle, texid: c_int) void {
        if (h.isRunning()) mjsim_updateTexture(h.sim.?, texid);
    }

    /// Close the window (if still open), release the viewer and free the handle.
    pub fn deinit(h: *Handle) void {
        if (h.isRunning()) mjsim_exit(h.sim.?);
        h.release.store(1, .release);
        h.thread.join();
        c.mjv_freeScene(&h.user_scn);
        h.gpa.destroy(h);
    }
};
