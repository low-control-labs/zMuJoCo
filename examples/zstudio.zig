const std = @import("std");
const zstudio = @import("zstudio");

// MuJoCo Studio, built on zstudio.
// USAGE: zstudio [--window_width N] [--window_height N] [--gfx API] [modelfile]

fn parseInt(args: []const [:0]const u8, i: usize) !c_int {
    if (i >= args.len) {
        std.log.err("missing value for {s}", .{args[i - 1]});
        return error.InvalidArguments;
    }
    return std.fmt.parseInt(c_int, args[i], 10);
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.arena.allocator();
    const args = try init.minimal.args.toSlice(gpa);

    var options: zstudio.Options = .{};
    var i: usize = 1;
    while (i < args.len) : (i += 1) {
        if (std.mem.eql(u8, args[i], "--window_width")) {
            options.window_width = try parseInt(args, i + 1);
            i += 1;
        } else if (std.mem.eql(u8, args[i], "--window_height")) {
            options.window_height = try parseInt(args, i + 1);
            i += 1;
        } else if (std.mem.eql(u8, args[i], "--gfx")) {
            if (i + 1 >= args.len) return error.InvalidArguments;
            options.gfx_mode = args[i + 1];
            i += 1;
        } else {
            options.model_file = args[i];
        }
    }

    try zstudio.launch(gpa, args, options);
}
