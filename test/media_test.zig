const std = @import("std");
const media = @import("media");
const testing = std.testing;

// media.writeImage writes a valid PNG whose pixels decode back unchanged.

test "write image round trip" {
    const width = 7;
    const height = 5;

    // test pattern
    var rgb: [3 * width * height]u8 = undefined;
    for (&rgb, 0..) |*p, i| p.* = @truncate(i * 37);

    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    var path_buf: [256]u8 = undefined;
    const path = try std.fmt.bufPrint(&path_buf, ".zig-cache/tmp/{s}/image.png", .{tmp.sub_path});
    try media.writeImage(testing.io, testing.allocator, path, &rgb, width, height);

    const png = try tmp.dir.readFileAlloc(testing.io, "image.png", testing.allocator, .unlimited);
    defer testing.allocator.free(png);

    // signature and IHDR
    try testing.expectEqualSlices(u8, "\x89PNG\r\n\x1a\n", png[0..8]);
    try testing.expectEqualSlices(u8, "IHDR", png[12..16]);
    try testing.expectEqual(@as(u32, width), std.mem.readInt(u32, png[16..20], .big));
    try testing.expectEqual(@as(u32, height), std.mem.readInt(u32, png[20..24], .big));
    try testing.expectEqual(@as(u8, 8), png[24]); // bit depth
    try testing.expectEqual(@as(u8, 2), png[25]); // RGB

    // IDAT follows IHDR (8 signature + 4 length + 4 type + 13 data + 4 crc)
    const idat_at = 8 + 4 + 4 + 13 + 4;
    const idat_len = std.mem.readInt(u32, png[idat_at..][0..4], .big);
    try testing.expectEqualSlices(u8, "IDAT", png[idat_at + 4 .. idat_at + 8]);
    const idat = png[idat_at + 8 ..][0..idat_len];

    // decompress, strip the filter byte of each row, compare
    var input: std.Io.Reader = .fixed(idat);
    var window: [std.compress.flate.max_window_len]u8 = undefined;
    var decompress: std.compress.flate.Decompress = .init(&input, .zlib, &window);
    const raw = try decompress.reader.allocRemaining(testing.allocator, .unlimited);
    defer testing.allocator.free(raw);
    try testing.expectEqual(@as(usize, height * (1 + 3 * width)), raw.len);
    for (0..height) |y| {
        const row = raw[y * (1 + 3 * width) ..][0 .. 1 + 3 * width];
        try testing.expectEqual(@as(u8, 0), row[0]);
        try testing.expectEqualSlices(u8, rgb[y * 3 * width ..][0 .. 3 * width], row[1..]);
    }
}
