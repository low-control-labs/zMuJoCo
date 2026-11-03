const std = @import("std");
const Io = std.Io;

// Image and video output for RGB frames, the Zig counterpart of the mediapy
// calls used in the Python notebooks (show_image, show_video).

/// Encodes RGB frames into a video file by piping them into ffmpeg, which
/// must be installed (mediapy has the same requirement).
pub const VideoWriter = struct {
    io: Io,
    child: std.process.Child,
    frame_len: usize,

    pub fn init(io: Io, path: []const u8, width: usize, height: usize, fps: f64) !VideoWriter {
        var size_buf: [32]u8 = undefined;
        var fps_buf: [32]u8 = undefined;
        const argv = [_][]const u8{
            "ffmpeg",        "-y",          "-loglevel", "error",
            "-f",            "rawvideo",    "-pixel_format", "rgb24",
            "-video_size",   try std.fmt.bufPrint(&size_buf, "{d}x{d}", .{ width, height }),
            "-framerate",    try std.fmt.bufPrint(&fps_buf, "{d}", .{fps}),
            "-i",            "-",
            "-pix_fmt",      "yuv420p",
            path,
        };
        const child = std.process.spawn(io, .{ .argv = &argv, .stdin = .pipe }) catch |err| {
            std.log.err("could not start ffmpeg ({t}), is it installed?", .{err});
            return err;
        };
        return .{ .io = io, .child = child, .frame_len = 3 * width * height };
    }

    /// Append one frame: RGB pixels, top row first.
    pub fn write(self: *VideoWriter, rgb: []const u8) !void {
        std.debug.assert(rgb.len == self.frame_len);
        try self.child.stdin.?.writeStreamingAll(self.io, rgb);
    }

    /// Close the stream and wait for ffmpeg to finish writing the file.
    pub fn finish(self: *VideoWriter) !void {
        // close stdin ourselves so ffmpeg sees end of input; wait() would only
        // close it after the child exits
        if (self.child.stdin) |stdin| {
            stdin.close(self.io);
            self.child.stdin = null;
        }
        const term = try self.child.wait(self.io);
        if (!term.success()) return error.FfmpegFailed;
    }
};

/// Write RGB pixels (top row first) as a PNG file.
pub fn writeImage(io: Io, gpa: std.mem.Allocator, path: []const u8, rgb: []const u8, width: usize, height: usize) !void {
    std.debug.assert(rgb.len == 3 * width * height);

    // image data: zlib stream of scanlines, each prefixed with filter type 0
    var idat: Io.Writer.Allocating = try .initCapacity(gpa, 64 * 1024);
    defer idat.deinit();
    {
        const window = try gpa.alloc(u8, std.compress.flate.max_window_len);
        defer gpa.free(window);
        var zlib = try std.compress.flate.Compress.init(&idat.writer, window, .zlib, .default);
        for (0..height) |y| {
            try zlib.writer.writeByte(0);
            try zlib.writer.writeAll(rgb[3 * width * y ..][0 .. 3 * width]);
        }
        try zlib.finish();
    }

    var png: Io.Writer.Allocating = try .initCapacity(gpa, idat.written().len + 64);
    defer png.deinit();
    try png.writer.writeAll("\x89PNG\r\n\x1a\n");

    var ihdr: [13]u8 = undefined;
    std.mem.writeInt(u32, ihdr[0..4], @intCast(width), .big);
    std.mem.writeInt(u32, ihdr[4..8], @intCast(height), .big);
    ihdr[8] = 8; // bit depth
    ihdr[9] = 2; // color type: RGB
    ihdr[10] = 0; // compression
    ihdr[11] = 0; // filter
    ihdr[12] = 0; // no interlace
    try writeChunk(&png.writer, "IHDR", &ihdr);
    try writeChunk(&png.writer, "IDAT", idat.written());
    try writeChunk(&png.writer, "IEND", "");

    const file = try Io.Dir.cwd().createFile(io, path, .{});
    defer file.close(io);
    try file.writeStreamingAll(io, png.written());
}

fn writeChunk(w: *Io.Writer, chunk_type: *const [4]u8, data: []const u8) !void {
    try w.writeInt(u32, @intCast(data.len), .big);
    try w.writeAll(chunk_type);
    try w.writeAll(data);
    var crc = std.hash.Crc32.init();
    crc.update(chunk_type);
    crc.update(data);
    try w.writeInt(u32, crc.final(), .big);
}
