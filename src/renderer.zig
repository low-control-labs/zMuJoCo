const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
const c = lib_mujoco.c;
const glfw = lib_mujoco.glfw;

// Offscreen renderer, a port of python/mujoco/renderer.py (GLFW backend):
// an invisible GLFW window provides the OpenGL context, rendering goes to the
// offscreen buffer and pixels are read back and flipped right side up.

pub const Renderer = struct {
    gpa: std.mem.Allocator,
    model: [*c]const c.mjModel,
    width: usize,
    height: usize,
    window: *glfw.Window,
    scene: c.mjvScene,
    context: c.mjrContext,
    option: c.mjvOption,
    free_camera: c.mjvCamera,
    rgb: []u8,
    row: []u8,

    pub const Error = error{ WidthTooLarge, HeightTooLarge };

    /// Width and height must fit in the model's offscreen buffer, set with
    /// <visual><global offwidth="..." offheight="..."/> in the model XML.
    pub fn init(gpa: std.mem.Allocator, m: [*c]const c.mjModel, height: usize, width: usize) !Renderer {
        // Python: 'Image width <w> > framebuffer width <offwidth>. Either reduce
        // the image width or specify a larger offscreen framebuffer in the
        // model XML: <visual><global offwidth="my_width"/></visual>'
        if (width > @as(usize, @intCast(m.*.vis.global.offwidth))) return Error.WidthTooLarge;
        if (height > @as(usize, @intCast(m.*.vis.global.offheight))) return Error.HeightTooLarge;

        // invisible window for the OpenGL context
        try glfw.init();
        glfw.windowHint(.visible, false);
        const window = try glfw.createWindow(@intCast(width), @intCast(height), "MuJoCo offscreen", null);
        errdefer glfw.destroyWindow(window);
        glfw.makeContextCurrent(window);

        const rgb = try gpa.alloc(u8, 3 * width * height);
        errdefer gpa.free(rgb);
        const row = try gpa.alloc(u8, 3 * width);
        errdefer gpa.free(row);

        var self: Renderer = .{
            .gpa = gpa,
            .model = m,
            .width = width,
            .height = height,
            .window = window,
            .scene = undefined,
            .context = undefined,
            .option = undefined,
            .free_camera = undefined,
            .rgb = rgb,
            .row = row,
        };
        c.mjv_defaultScene(&self.scene);
        c.mjv_makeScene(m, &self.scene, 10000);
        c.mjr_defaultContext(&self.context);
        c.mjr_makeContext(m, &self.context, c.mjFONTSCALE_150);
        c.mjr_setBuffer(c.mjFB_OFFSCREEN, &self.context);
        c.mjv_defaultOption(&self.option);
        return self;
    }

    pub fn deinit(self: *Renderer) void {
        glfw.makeContextCurrent(self.window);
        c.mjr_freeContext(&self.context);
        c.mjv_freeScene(&self.scene);
        glfw.makeContextCurrent(null);
        glfw.destroyWindow(self.window);
        self.gpa.free(self.rgb);
        self.gpa.free(self.row);
    }

    /// Camera for updateScene, as the camera argument of Python's update_scene:
    /// the default free camera (-1), a model camera by index or name, or a
    /// caller-owned mjvCamera.
    pub const Camera = union(enum) {
        free,
        id: c_int,
        name: []const u8,
        custom: *c.mjvCamera,
    };

    pub const CameraError = error{ UnknownCamera, CameraOutOfRange };

    /// Update the scene from data. A null option uses the default visualization
    /// options.
    pub fn updateScene(self: *Renderer, d: [*c]c.mjData, camera: Camera, option: ?*const c.mjvOption) !void {
        const cam: *c.mjvCamera = switch (camera) {
            .custom => |cam| cam,
            .free, .id, .name => blk: {
                var camera_id: c_int = -1;
                switch (camera) {
                    .id => |id| camera_id = id,
                    .name => |name| {
                        var name_buf: [256]u8 = undefined;
                        const name_z = std.fmt.bufPrintSentinel(&name_buf, "{s}", .{name}, 0) catch return CameraError.UnknownCamera;
                        camera_id = c.mj_name2id(self.model, c.mjOBJ_CAMERA, name_z);
                        // Python: 'The camera "<name>" does not exist.'
                        if (camera_id == -1) return CameraError.UnknownCamera;
                    },
                    else => {},
                }
                // Python: 'The camera id <id> is out of range [-1, <ncam>).'
                if (camera_id < -1 or camera_id >= self.model.*.ncam) return CameraError.CameraOutOfRange;

                // default free camera, or the model's camera
                c.mjv_defaultCamera(&self.free_camera);
                self.free_camera.fixedcamid = camera_id;
                if (camera_id == -1) {
                    self.free_camera.type = c.mjCAMERA_FREE;
                    c.mjv_defaultFreeCamera(self.model, &self.free_camera);
                } else {
                    self.free_camera.type = c.mjCAMERA_FIXED;
                }
                break :blk &self.free_camera;
            },
        };
        c.mjv_updateScene(self.model, d, option orelse &self.option, null, cam, c.mjCAT_ALL, &self.scene);
    }

    /// Render the scene; returns RGB pixels (height x width x 3), top row first.
    /// The slice is owned by the renderer and overwritten by the next call.
    pub fn render(self: *Renderer) []const u8 {
        return self.renderInto(self.rgb) catch unreachable;
    }

    /// Render the scene into out (height x width x 3), like render(out=...) in
    /// Python; returns out.
    pub fn renderInto(self: *Renderer, out: []u8) ![]u8 {
        // Python: 'Expected `out.shape == (height, width, 3)`'
        if (out.len != 3 * self.width * self.height) return error.WrongOutputSize;
        glfw.makeContextCurrent(self.window);
        const rect: c.mjrRect = .{
            .left = 0,
            .bottom = 0,
            .width = @intCast(self.width),
            .height = @intCast(self.height),
        };
        c.mjr_render(rect, &self.scene, &self.context);
        c.mjr_readPixels(out.ptr, null, rect, &self.context);

        // OpenGL reads bottom-up, flip like np.flipud
        const stride = 3 * self.width;
        for (0..self.height / 2) |y| {
            const top = out[y * stride ..][0..stride];
            const bottom = out[(self.height - 1 - y) * stride ..][0..stride];
            @memcpy(self.row, top);
            @memcpy(top, bottom);
            @memcpy(bottom, self.row);
        }
        return out;
    }
};
