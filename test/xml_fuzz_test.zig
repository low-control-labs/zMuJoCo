const std = @import("std");
const c = @import("lib_z_mujoco").c;
const testing = std.testing;

// Fuzzes MuJoCo's XML parser (tinyxml2 + MuJoCo's reader), model compiler and
// a few simulation steps with mutated XML. Any input may be rejected with an
// error message; it must never crash or hit undefined behaviour.
//   zig build test-xml_fuzz           runs the seed corpus once
//   zig build test-xml_fuzz --fuzz    fuzzes (web interface for progress)
// Note: MuJoCo's fatal errors (mju_error) exit the process by default, which the
// fuzzer reports as a crash; such reports need checking before calling them bugs.

const corpus = [_][]const u8{
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
    ,
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
    ,
    \\<mujoco>
    \\  <worldbody>
    \\    <body name="1" mocap="true"/>
    \\    <body name="2" mocap="true"/>
    \\  </worldbody>
    \\  <sensor>
    \\    <framepos objtype="xbody" objname="1"/>
    \\    <framequat objtype="xbody" objname="1"/>
    \\  </sensor>
    \\</mujoco>
    ,
    \\<mujoco>
    \\  <worldbody>
    \\    <body>
    \\      <joint name="ball" type="ball"/>
    \\      <geom type="sphere" size=".1" pos="0 0 -.5"/>
    \\      <body pos="0 0 -.5">
    \\        <joint name="slide" type="slide" axis="1 0 0"/>
    \\        <geom type="cylinder" size=".05 .1"/>
    \\      </body>
    \\    </body>
    \\  </worldbody>
    \\  <tendon>
    \\    <fixed name="t"><joint joint="slide" coef="1"/></fixed>
    \\  </tendon>
    \\  <equality>
    \\    <connect body1="world" anchor="0 0 0"/>
    \\  </equality>
    \\</mujoco>
    ,
    \\<mujoco>
    \\</mujoco>
    ,
};

// MuJoCo warnings (e.g. diverging garbage models) are expected: keep them quiet
fn ignoreWarning(_: [*c]const u8) callconv(.c) void {}

fn parseOne(_: void, smith: *testing.Smith) anyerror!void {
    var buf: [4096:0]u8 = undefined;
    const len = smith.slice(&buf);
    buf[len] = 0;

    var err: [1000]u8 = undefined;
    const spec = c.mj_parseXMLString(&buf, null, &err, err.len);
    if (spec == null) return;
    defer c.mj_deleteSpec(spec);
    const m = c.mj_compile(spec, null);
    if (m == null) return;
    defer c.mj_deleteModel(m);
    const d = c.mj_makeData(m);
    if (d == null) return;
    defer c.mj_deleteData(d);
    for (0..10) |_| c.mj_step(m, d);
}

test "fuzz xml parser and compiler" {
    c.mju_user_warning = ignoreWarning;
    defer c.mju_user_warning = null;
    // `--fuzz` crashes in Zig 0.17's own fuzzer ("start index 1 is larger than
    // end index 0", lib/fuzzer.zig copy mutation), also with a minimal pure-Zig
    // fuzz test; without --fuzz the seed corpus runs normally
    try testing.fuzz({}, parseOne, .{ .corpus = &corpus });
}
