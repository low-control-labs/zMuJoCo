const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
pub const c = lib_mujoco.c;

// MuJoCo Studio (studio/ of the MuJoCo sources), on the C layer in
// zig/mujoco/studio.cc. The studio sources are used unmodified.
//   launch: studio runs the physics and the UI (blocking)

extern fn mjstudio_launch(
    argc: c_int,
    argv: [*]const ?[*:0]const u8,
    window_width: c_int,
    window_height: c_int,
    title: ?[*:0]const u8,
    model_file: ?[*:0]const u8,
    gfx_mode: ?[*:0]const u8,
) c_int;

pub const Options = struct {
    window_width: c_int = 1400,
    window_height: c_int = 720,
    title: [:0]const u8 = "MuJoCo Studio",
    /// model loaded on startup; null starts without one
    model_file: ?[:0]const u8 = null,
    /// graphics API; null for the default
    gfx_mode: ?[:0]const u8 = null,
};

pub const Error = error{StudioFailed};

/// Launch studio and block until its window is closed. args is the command
/// line, as studio parses it.
pub fn launch(gpa: std.mem.Allocator, args: []const [:0]const u8, options: Options) !void {
    const argv = try gpa.alloc(?[*:0]const u8, args.len + 1);
    defer gpa.free(argv);
    for (args, 0..) |arg, i| argv[i] = arg.ptr;
    argv[args.len] = null;

    const rc = mjstudio_launch(
        @intCast(args.len),
        argv.ptr,
        options.window_width,
        options.window_height,
        options.title.ptr,
        if (options.model_file) |model| model.ptr else null,
        if (options.gfx_mode) |gfx| gfx.ptr else null,
    );
    if (rc != 0) return Error.StudioFailed;
}
