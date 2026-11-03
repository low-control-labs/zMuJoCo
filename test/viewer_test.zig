const std = @import("std");
const zsimulate = @import("zsimulate");
const c = zsimulate.c;
const testing = std.testing;

// The simulate viewer through zsimulate's passive mode. viewer_test.py only
// checks that viewer.launch exists; these load models into the viewer and draw
// frames, which runs simulate's Load, LoadOnRenderThread, Sync and RenderLoop.
// Needs a display (zig build test-display); run from the repository root.

fn loadModel(path: [*:0]const u8) ![*c]c.mjModel {
    var load_error: [1000]u8 = undefined;
    const m = c.mj_loadXML(path, null, &load_error, load_error.len);
    if (m == null) return error.ModelLoadFailed;
    return m;
}

// step and sync for n frames, giving the render thread time to draw
fn run(viewer: *zsimulate.Handle, m: [*c]c.mjModel, d: [*c]c.mjData, nframe: usize, steps_per_frame: usize) void {
    for (0..nframe) |_| {
        for (0..steps_per_frame) |_| c.mj_step(m, d);
        viewer.sync(false);
        testing.io.sleep(.fromMilliseconds(16), .awake) catch {};
    }
}

test "passive viewer with the humanoid" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();
    try testing.expect(viewer.isRunning());

    run(viewer, m, d, 60, 4);
    try testing.expect(viewer.isRunning());
    try testing.expect(d.*.time > 0);
}

test "camera and options through the handle" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();

    // like viewer.cam / viewer.opt under viewer.lock() in Python
    viewer.lock();
    c.mjv_defaultFreeCamera(m, viewer.cam());
    viewer.opt().flags[c.mjVIS_CONTACTFORCE] = 1;
    viewer.unlock();
    run(viewer, m, d, 10, 4);

    try testing.expectApproxEqAbs(1.5 * m.*.stat.extent, viewer.cam().distance, 1e-9);
    try testing.expectEqual(@as(u8, 1), viewer.opt().flags[c.mjVIS_CONTACTFORCE]);
}

test "every model in the passive viewer" {
    var dir = try std.Io.Dir.cwd().openDir(testing.io, "model", .{ .iterate = true });
    defer dir.close(testing.io);
    var walker = try dir.walk(testing.allocator);
    defer walker.deinit();

    var shown: usize = 0;
    while (try walker.next(testing.io)) |entry| {
        if (entry.kind != .file or !std.mem.endsWith(u8, entry.path, ".xml")) continue;

        var path_buf: [std.fs.max_path_bytes]u8 = undefined;
        const path = try std.fmt.bufPrintSentinel(&path_buf, "model/{s}", .{entry.path}, 0);
        // files that are not standalone models (e.g. <include> fragments) are skipped
        const m = loadModel(path) catch continue;
        defer c.mj_deleteModel(m);
        const d = c.mj_makeData(m);
        defer c.mj_deleteData(d);
        if (m.*.nkey > 0) c.mj_resetDataKeyframe(m, d, 0);

        const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
        defer viewer.deinit();
        run(viewer, m, d, 5, 2);
        shown += 1;
    }

    // guard against the test silently doing nothing (wrong working directory)
    try testing.expect(shown >= 20);
}

// The tests below call the handle from a second thread while the main thread
// steps and syncs, which is what simulate's request/lock logic is for. They are
// meant for zig build test-viewer -Dsanitize-thread.

const Worker = struct {
    viewer: *zsimulate.Handle,
    stop: std.atomic.Value(bool) = .init(false),

    // text, figure and image requests: each waits for the render thread
    fn overlays(w: *Worker) void {
        var fig: c.mjvFigure = undefined;
        c.mjv_defaultFigure(&fig);
        const rect: c.mjrRect = .{ .left = 0, .bottom = 0, .width = 100, .height = 100 };
        const rgb: [4 * 4 * 3]u8 = @splat(128);
        while (!w.stop.load(.acquire)) {
            w.viewer.setTexts(&.{.{ .text1 = "stress" }});
            w.viewer.setFigures(&.{.{ .viewport = rect, .figure = &fig }});
            w.viewer.setImages(&.{.{ .viewport = rect, .rgb = &rgb, .width = 4, .height = 4 }});
            w.viewer.clearTexts();
            w.viewer.clearFigures();
            w.viewer.clearImages();
        }
    }

    // camera and visualization options under the viewer lock
    fn camera(w: *Worker) void {
        while (!w.stop.load(.acquire)) {
            w.viewer.lock();
            w.viewer.cam().azimuth += 1;
            w.viewer.opt().flags[c.mjVIS_CONTACTPOINT] ^= 1;
            w.viewer.unlock();
            std.Thread.yield() catch {};
        }
    }

    // render flags in the user scene; Sync() picks up the change and sets
    // pending_.ui_update_rendering, which Render() reads and clears
    fn sceneFlags(w: *Worker) void {
        while (!w.stop.load(.acquire)) {
            w.viewer.lock();
            w.viewer.userScn().flags[c.mjRND_SHADOW] ^= 1;
            w.viewer.unlock();
            std.Thread.yield() catch {};
        }
    }
};

fn runWithWorker(comptime func: anytype, viewer: *zsimulate.Handle, m: [*c]c.mjModel, d: [*c]c.mjData) !void {
    var worker: Worker = .{ .viewer = viewer };
    const thread = try std.Thread.spawn(.{}, func, .{&worker});
    run(viewer, m, d, 60, 4);
    worker.stop.store(true, .release);
    thread.join();
    try testing.expect(viewer.isRunning());
}

test "text, figure and image requests from another thread" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();
    try runWithWorker(Worker.overlays, viewer, m, d);
}

test "user scene render flags from another thread" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();
    try runWithWorker(Worker.sceneFlags, viewer, m, d);
}

test "camera and options from another thread" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();
    try runWithWorker(Worker.camera, viewer, m, d);
}

test "asset updates while rendering" {
    // gothic.xml has meshes and textures; each id is re-uploaded while drawing
    const m = try loadModel("model/arch/gothic.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);
    try testing.expect(m.*.nmesh > 0 or m.*.ntex > 0 or m.*.nhfield > 0);

    const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
    defer viewer.deinit();

    for (0..5) |_| {
        for (0..@intCast(m.*.nhfield)) |i| viewer.updateHField(@intCast(i));
        for (0..@intCast(m.*.nmesh)) |i| viewer.updateMesh(@intCast(i));
        for (0..@intCast(m.*.ntex)) |i| viewer.updateTexture(@intCast(i));
        run(viewer, m, d, 5, 2);
    }
    try testing.expect(viewer.isRunning());
}

test "launch and close repeatedly, with a request pending" {
    const m = try loadModel("model/humanoid/humanoid.xml");
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    for (0..5) |i| {
        const viewer = try zsimulate.launchPassive(testing.allocator, m, d, .{});
        // close right after a request, before the render thread may have taken it
        if (i % 2 == 0) viewer.setTexts(&.{.{ .text1 = "closing" }});
        viewer.deinit();
    }
}
