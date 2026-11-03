const std = @import("std");
const c = @import("lib_z_mujoco").c;
const Renderer = @import("renderer").Renderer;
const testing = std.testing;

// Port of python/mujoco/renderer_test.py, plus checks of the offscreen size
// limit and of the image orientation. Needs a display (zig build test-display).

const camera_xml =
    \\<mujoco>
    \\  <worldbody>
    \\    <camera name="a"/>
    \\  </worldbody>
    \\</mujoco>
;

const box_xml =
    \\<mujoco>
    \\  <worldbody>
    \\    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    \\    <geom name="white_box" type="box" size="1 1 1" rgba="1 1 1 1"/>
    \\  </worldbody>
    \\</mujoco>
;

// the closeup camera looks along +y with +z up: red box above, blue box below
const orientation_xml =
    \\<mujoco>
    \\  <worldbody>
    \\    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    \\    <geom name="red_top" type="box" pos="0 0 .8" size=".4 .4 .4" rgba="1 0 0 1"/>
    \\    <geom name="blue_bottom" type="box" pos="0 0 -.8" size=".4 .4 .4" rgba="0 0 1 1"/>
    \\  </worldbody>
    \\</mujoco>
;

const Scene = struct {
    m: [*c]c.mjModel,
    d: [*c]c.mjData,

    fn load(xml: []const u8) !Scene {
        const z = try testing.allocator.dupeSentinel(u8, xml, 0);
        defer testing.allocator.free(z);
        var err: [1000]u8 = undefined;
        const spec = c.mj_parseXMLString(z, null, &err, err.len);
        if (spec == null) return error.ModelParseFailed;
        defer c.mj_deleteSpec(spec);
        const m = c.mj_compile(spec, null);
        if (m == null) return error.ModelCompileFailed;
        const d = c.mj_makeData(m);
        c.mj_forward(m, d);
        return .{ .m = m, .d = d };
    }

    fn deinit(s: Scene) void {
        c.mj_deleteData(s.d);
        c.mj_deleteModel(s.m);
    }
};

fn anyNonZero(pixels: []const u8) bool {
    for (pixels) |p| if (p > 0) return true;
    return false;
}

test "renderer unknown camera name" {
    const s = try Scene.load(camera_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try testing.expectError(error.UnknownCamera, renderer.updateScene(s.d, .{ .name = "b" }, null));
}

test "renderer camera under range" {
    const s = try Scene.load(camera_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try testing.expectError(error.CameraOutOfRange, renderer.updateScene(s.d, .{ .id = -2 }, null));
}

test "renderer camera over range" {
    const s = try Scene.load(camera_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try testing.expectError(error.CameraOutOfRange, renderer.updateScene(s.d, .{ .id = 1 }, null));
}

test "renderer renders scene" {
    const s = try Scene.load(box_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try renderer.updateScene(s.d, .{ .name = "closeup" }, null);
    try testing.expect(anyNonZero(renderer.render()));
}

test "renderer output without out" {
    const s = try Scene.load(box_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try renderer.updateScene(s.d, .{ .name = "closeup" }, null);

    // render() reuses the renderer's buffer (Python returns a new array), so
    // keep a copy of the previous frame
    const previous = try testing.allocator.dupe(u8, renderer.render());
    defer testing.allocator.free(previous);

    const colors = [_][4]f32{ .{ 1, 0, 0, 1 }, .{ 0, 1, 0, 1 }, .{ 0, 0, 1, 1 } };
    for (colors) |color| {
        @memcpy(s.m.*.geom_rgba[0..4], &color);
        c.mj_forward(s.m, s.d);
        try renderer.updateScene(s.d, .{ .name = "closeup" }, null);
        const pixels = renderer.render();

        // pixels change over steps
        try testing.expect(!std.mem.eql(u8, previous, pixels));
        @memcpy(previous, pixels);
    }
}

test "renderer output with out" {
    const s = try Scene.load(box_xml);
    defer s.deinit();
    var renderer = try Renderer.init(testing.allocator, s.m, 50, 50);
    defer renderer.deinit();
    try renderer.updateScene(s.d, .{ .name = "closeup" }, null);

    var out: [50 * 50 * 3]u8 = @splat(0);
    const pixels = try renderer.renderInto(&out);

    // pixels refer to the same `out` buffer
    try testing.expectEqual(@as([*]u8, &out), pixels.ptr);
    try testing.expect(anyNonZero(&out));

    // wrong size
    var small: [10 * 10 * 3]u8 = @splat(0);
    try testing.expectError(error.WrongOutputSize, renderer.renderInto(&small));
}

// not in renderer_test.py: Python raises for sizes beyond the offscreen buffer
test "renderer size beyond offscreen buffer" {
    const s = try Scene.load(box_xml);
    defer s.deinit();
    const offwidth: usize = @intCast(s.m.*.vis.global.offwidth);
    const offheight: usize = @intCast(s.m.*.vis.global.offheight);
    try testing.expectError(error.WidthTooLarge, Renderer.init(testing.allocator, s.m, 50, offwidth + 1));
    try testing.expectError(error.HeightTooLarge, Renderer.init(testing.allocator, s.m, offheight + 1, 50));
}

// not in renderer_test.py: the image is right side up (the np.flipud step)
test "renderer image orientation" {
    const s = try Scene.load(orientation_xml);
    defer s.deinit();
    const size = 50;
    var renderer = try Renderer.init(testing.allocator, s.m, size, size);
    defer renderer.deinit();
    try renderer.updateScene(s.d, .{ .name = "closeup" }, null);
    const pixels = renderer.render();

    // sum of red minus blue in the top and bottom halves
    var top: i64 = 0;
    var bottom: i64 = 0;
    for (0..size) |y| {
        for (0..size) |x| {
            const p = pixels[3 * (y * size + x) ..][0..3];
            const red_minus_blue = @as(i64, p[0]) - @as(i64, p[2]);
            if (y < size / 2) top += red_minus_blue else bottom += red_minus_blue;
        }
    }
    try testing.expect(top > 0); // red box on top
    try testing.expect(bottom < 0); // blue box at the bottom
}
