const std = @import("std");
const c = @import("lib_z_mujoco").c;
const testing = std.testing;

// Load, compile and step every model in model/ (run from the repository root).
// Run in a Debug build, this exercises the XML parser, the compiler, collision
// and the engine across many features with Zig's undefined-behaviour checks
// enabled for all the C/C++ code. Files that are not standalone models (e.g.
// <include> fragments) fail to load and are skipped.

const nstep = 200;

test "load and step all models" {
    var dir = try std.Io.Dir.cwd().openDir(testing.io, "model", .{ .iterate = true });
    defer dir.close(testing.io);
    var walker = try dir.walk(testing.allocator);
    defer walker.deinit();

    var loaded: usize = 0;
    var skipped: usize = 0;
    while (try walker.next(testing.io)) |entry| {
        if (entry.kind != .file or !std.mem.endsWith(u8, entry.path, ".xml")) continue;

        var path_buf: [std.fs.max_path_bytes]u8 = undefined;
        const path = try std.fmt.bufPrintSentinel(&path_buf, "model/{s}", .{entry.path}, 0);
        var load_error: [1000]u8 = undefined;
        const m = c.mj_loadXML(path, null, &load_error, load_error.len);
        if (m == null) {
            std.log.info("skipped {s}: {s}", .{ path, std.mem.sliceTo(&load_error, 0) });
            skipped += 1;
            continue;
        }
        defer c.mj_deleteModel(m);
        const d = c.mj_makeData(m);
        defer c.mj_deleteData(d);

        // from the first keyframe if there is one, otherwise the default state
        if (m.*.nkey > 0) c.mj_resetDataKeyframe(m, d, 0);
        for (0..nstep) |_| c.mj_step(m, d);
        c.mj_forward(m, d);
        loaded += 1;
    }

    std.log.info("stepped {d} models, skipped {d} files", .{ loaded, skipped });
    // guard against the test silently doing nothing (wrong working directory)
    try testing.expect(loaded >= 20);
}
