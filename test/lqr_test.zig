const std = @import("std");
const c = @import("lib_z_mujoco").c;
const Lqr = @import("lqr").Lqr;
const testing = std.testing;

// The LQR controller of python/LQR.ipynb keeps the humanoid balanced on one leg
// (run from the repository root).

test "humanoid stays balanced" {
    var load_error: [1000]u8 = undefined;
    const m = c.mj_loadXML("model/humanoid/humanoid.xml", null, &load_error, load_error.len);
    if (m == null) return error.ModelLoadFailed;
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    defer c.mj_deleteData(d);

    var arena_state: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena_state.deinit();
    var lqr = try Lqr.init(arena_state.allocator(), m, d);

    // finite gains
    for (lqr.k) |k| try testing.expect(std.math.isFinite(k));

    // closed loop for 5 seconds, like the notebook's first LQR video
    lqr.setPose(m, d);
    while (d.*.time < 5) {
        lqr.control(m, d);
        c.mj_step(m, d);
    }

    // still standing: the root height stays close to the setpoint
    try testing.expectApproxEqAbs(lqr.qpos0[2], d.*.qpos[2], 0.05);
}
