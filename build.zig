const std = @import("std");

// The fetched MuJoCo source package (see build.zig.zon). Set first in build().
// The archive unpacks to a single directory inside the package.
const mujoco_prefix = "mujoco-a6289c13780bfb876d496bf76ba9ac6a91b50760";
const MujocoDep = struct {
    dep: *std.Build.Dependency,

    fn path(self: MujocoDep, sub: []const u8) std.Build.LazyPath {
        if (std.mem.eql(u8, sub, ".")) return self.dep.path(mujoco_prefix);
        return self.dep.path(self.dep.builder.fmt("{s}/{s}", .{ mujoco_prefix, sub }));
    }

    fn openDir(self: MujocoDep, io: std.Io, sub: []const u8, options: std.Io.Dir.OpenOptions) !std.Io.Dir {
        return self.dep.builder.root.openDir(io, self.dep.builder.fmt("{s}/{s}", .{ mujoco_prefix, sub }), options);
    }
};
var mujoco_dep: MujocoDep = undefined;

// base flags followed by extra flags (an optional flag can't sit in a fixed list)
fn withFlags(b: *std.Build, base: []const []const u8, extra: []const []const u8) []const []const u8 {
    return std.mem.concat(b.allocator, []const u8, &.{ base, extra }) catch @panic("OOM");
}

// === Studio ===
// Filament, SDL2, libwebp and abseil are built by CMake with `zig cc` / `zig c++`
// (Filament also needs its host tools matc, resgen, cmgen first); MuJoCo's
// Filament layer, studio itself, Dear ImGui and ImPlot are compiled by Zig.

const FilamentTools = struct {
    matc: std.Build.LazyPath,
    /// CMake build directory of the tools, with the export for the libraries
    dir: std.Build.LazyPath,
};

// Filament's host tools, built with CMake (it runs the Python generators of the
// vendored SPIRV-Tools itself).
fn addFilamentTools(b: *std.Build) ?FilamentTools {
    const dep = b.lazyDependency("filament", .{}) orelse return null;

    const script =
        \\set -e
        \\# Filament wants the export directory relative to its source tree
        \\REL=$(realpath --relative-to="$1" "$2")
        \\cmake -S "$1" -B "$2" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        \\  -DFILAMENT_SKIP_SAMPLES=ON -DFILAMENT_SKIP_SDL2=ON \
        \\  -DFILAMENT_EXPORT_PREBUILT_EXECUTABLES_DIR="$REL" \
        \\  -DFILAMENT_ENABLE_EXPERIMENTAL_GCC_SUPPORT=ON \
        \\  -DCMAKE_CXX_FLAGS="-include cstring -include optional -include cstdint"
        \\cmake --build "$2" --target matc resgen glslminifier cmgen
    ;
    const run = b.addSystemCommand(&.{ "sh", "-c", script, "sh" });
    // the tarball unpacks to a single directory inside the package
    run.addDirectoryArg(dep.path(""));
    const build_dir = run.addOutputDirectoryArg("filament-tools");

    b.step("filament-tools", "Build Filament's matc, resgen and glslminifier with CMake").dependOn(&run.step);
    return .{ .matc = build_dir.path(b, "tools/matc/matc"), .dir = build_dir };
}

// The Filament libraries, OpenGL only, using the tools above through Filament's
// prebuilt executables mechanism. Returns the CMake build directory.
fn addFilamentLibs(b: *std.Build, tools: FilamentTools, optimize: std.lang.Optimize) ?std.Build.LazyPath {
    const dep = b.lazyDependency("filament", .{}) orelse return null;

    const script =
        \\set -e
        \\export CC="$3 cc" CXX="$3 c++" ASM="$3 cc"
        \\REL=$(realpath --relative-to="$1" "$5")
        \\cmake -S "$1" -B "$2" -G Ninja -DCMAKE_BUILD_TYPE="$4" \
        \\  -DFILAMENT_SKIP_SAMPLES=ON -DFILAMENT_SKIP_SDL2=ON \
        \\  -DFILAMENT_IMPORT_PREBUILT_EXECUTABLES_DIR="$REL" \
        \\  -DFILAMENT_ENABLE_MATDBG=OFF -DFILAMENT_ENABLE_FGVIEWER=OFF \
        \\  -DFILAMENT_SUPPORTS_VULKAN=OFF \
        \\  -DCCACHE_PROGRAM=CCACHE_PROGRAM-NOTFOUND \
        \\  -DCMAKE_C_FLAGS=-Wno-unused-command-line-argument \
        \\  -DCMAKE_CXX_FLAGS=-Wno-unused-command-line-argument
        \\cmake --build "$2" --target filament image ktxreader
    ;
    const run = b.addSystemCommand(&.{ "sh", "-c", script, "sh" });
    run.addDirectoryArg(dep.path(""));
    const build_dir = run.addOutputDirectoryArg("filament-libs");
    run.addArg(b.graph.zig_exe);
    run.addArg(if (optimize == .debug) "Debug" else "Release");
    run.addDirectoryArg(tools.dir);

    b.step("filament-libs", "Build the Filament libraries with CMake and zig cc").dependOn(&run.step);
    return build_dir;
}

// A dependency built as static libraries by CMake with `zig cc` / `zig c++`.
// With `merge`, the archives matching that name are also merged into libmerged.a.
// Returns the CMake build directory.
fn addCmakeStatic(
    b: *std.Build,
    optimize: std.lang.Optimize,
    dep_name: []const u8,
    step_name: []const u8,
    cmake_args: []const u8,
    targets: []const u8,
    merge: ?[]const u8,
) ?std.Build.LazyPath {
    const dep = b.lazyDependency(dep_name, .{}) orelse return null;

    const script =
        \\set -e
        \\export CC="$3 cc" CXX="$3 c++" ASM="$3 cc"
        \\# the archive may unpack to a single directory inside the package
        \\SRC="$1"
        \\[ -f "$SRC/CMakeLists.txt" ] || SRC=$(echo "$1"/*/)
        \\cmake -S "$SRC" -B "$2" -G Ninja -DCMAKE_BUILD_TYPE="$4" \
        \\  -DCMAKE_C_FLAGS=-Wno-unused-command-line-argument \
        \\  -DCMAKE_CXX_FLAGS=-Wno-unused-command-line-argument $5
        \\cmake --build "$2" --target $6
        \\[ -z "$7" ] || { echo "create $2/libmerged.a"; for f in $(find "$2" -name "$7"); do echo "addlib $f"; done; echo save; echo end; } | ar -M
    ;
    const run = b.addSystemCommand(&.{ "sh", "-c", script, "sh" });
    run.addDirectoryArg(dep.path(""));
    const build_dir = run.addOutputDirectoryArg(step_name);
    run.addArg(b.graph.zig_exe);
    run.addArg(if (optimize == .debug) "Debug" else "Release");
    run.addArg(cmake_args);
    run.addArg(targets);
    run.addArg(merge orelse "");

    b.step(step_name, b.fmt("Build {s} with CMake and zig cc", .{dep_name})).dependOn(&run.step);
    return build_dir;
}

const StudioLibs = struct {
    /// CMake build directories
    filament: std.Build.LazyPath,
    sdl2: std.Build.LazyPath,
    webp: std.Build.LazyPath,
    abseil: std.Build.LazyPath,
};

// The sources of studio for a module: MuJoCo's Filament layer
// (src/render/filament), studio's own sources, Dear ImGui with the SDL2 and
// OpenGL3 backends and ImPlot, compiled here and linked against the libraries
// built with CMake. The native binary uses studio/main.cc (abseil flags); with
// `wrapper`, the C layer in zig/mujoco/studio.cc takes its place, for zstudio.
fn addStudioSources(
    b: *std.Build,
    module: *std.Build.Module,
    optimize: std.lang.Optimize,
    mujoco_lib: *std.Build.Step.Compile,
    libs: StudioLibs,
    wrapper: bool,
    plugin_files: []const []const u8,
) !void {
    module.addCSourceFiles(.{ .root = mujoco_dep.path("."), .files = plugin_files, .flags = &.{"-std=c++20"} });
    const filament_dep = b.lazyDependency("filament", .{}) orelse return;
    const imgui_dep = b.lazyDependency("imgui", .{}) orelse return;
    const implot_dep = b.lazyDependency("implot", .{}) orelse return;
    const webp_dep = b.lazyDependency("libwebp", .{}) orelse return;
    const abseil_dep = b.lazyDependency("abseil", .{}) orelse return;

    // every archive unpacks to a single directory inside the package
    const filament = filament_dep.path("");
    const imgui = imgui_dep.path("");
    const implot = implot_dep.path("");
    const webp_include = webp_dep.path("src");
    const abseil_include = abseil_dep.path("");

    const flags: []const []const u8 = &.{
        "-std=c++20",
        "-DFILAMENT_SUPPORTS_OPENGL",
        "-DFILAMENT_SUPPORTS_X11",
        "-DFILAMENT_SUPPORTS_XCB",
        "-DFILAMENT_SUPPORTS_XLIB",
        "-DUSING_GENERATED_CONFIG_H",
        // as Filament itself was built
        "-DFILAMENT_ENABLE_MATDBG=0",
        "-DFILAMENT_ENABLE_FGVIEWER=0",
        // empty structs of src/render/filament/core/types.h
        "-Wno-extern-c-compat",
    };

    // MuJoCo's Filament layer
    module.addCSourceFiles(.{
        .root = mujoco_dep.path("src/render/filament"),
        .files = &.{
            "mjrfilament.cc",
            "core/builtins.cc",
            "core/color_grading_options.cc",
            "core/filament_context.cc",
            "core/filament_platform_factory.cc",
            "core/imgui_editor.cc",
            "core/light.cc",
            "core/material_manager.cc",
            "core/mesh.cc",
            "core/object_manager.cc",
            "core/outliner.cc",
            "core/reflection_manager.cc",
            "core/render_target.cc",
            "core/renderable.cc",
            "core/scene_view.cc",
            "core/texture.cc",
            "support/filament_util.cc",
            "support/imgui_bridge.cc",
            "support/mesh_util.cc",
            "support/model_decorations.cc",
            "support/model_lights.cc",
            "support/model_renderables.cc",
            "support/model_resources.cc",
            "support/renderer.cc",
        },
        .flags = flags,
    });

    // studio: the platform library of its CMake (Linux) and the executable
    module.addCSourceFiles(.{
        .root = mujoco_dep.path("studio"),
        .files = &.{
            "hal/filament_renderer.cc",
            "hal/graphics_mode.cc",
            "hal/window.cc",
            "io/resources.cc",
            "render/model_scene.cc",
            "sim/model_holder.cc",
            "sim/sim_history.cc",
            "sim/sim_profiler.cc",
            "sim/step_control.cc",
            "ux/console.cc",
            "ux/fonts.cc",
            "ux/gui.cc",
            "ux/gui_helpers.cc",
            "ux/gui_spec.cc",
            "ux/imgui_widgets.cc",
            "ux/interaction.cc",
            "ux/picture_gui.cc",
            "ux/plugin.cc",
            "ux/spec_editor.cc",
            "ux/object_launcher_plugin.cc",
            "ux/file_dialog_zenity.cc",
            "app.cc",
            "launcher.cc",
        },
        .flags = flags,
    });
    if (wrapper) {
        module.addCSourceFiles(.{ .files = &.{"mujoco/studio.cc"}, .flags = flags });
    } else {
        module.addCSourceFiles(.{ .root = mujoco_dep.path("studio"), .files = &.{"main.cc"}, .flags = flags });
    }

    // Dear ImGui (core, SDL2 and OpenGL3 backends, stdlib helpers) and ImPlot
    module.addCSourceFiles(.{
        .root = imgui,
        .files = &.{
            "imgui.cpp",
            "imgui_draw.cpp",
            "imgui_tables.cpp",
            "imgui_widgets.cpp",
            "imgui_demo.cpp",
            "backends/imgui_impl_sdl2.cpp",
            "backends/imgui_impl_opengl3.cpp",
            "misc/cpp/imgui_stdlib.cpp",
        },
        .flags = flags,
    });
    module.addCSourceFiles(.{
        .root = implot,
        .files = &.{ "implot.cpp", "implot_items.cpp", "implot_demo.cpp" },
        .flags = flags,
    });

    module.addIncludePath(mujoco_dep.path("."));
    module.addIncludePath(mujoco_dep.path("include"));
    module.addIncludePath(mujoco_dep.path("src"));
    module.addIncludePath(imgui);
    module.addIncludePath(implot);
    // SDL's headers as copied by its CMake build, with the SDL_config.h it
    // generated (it enables the X11 driver); the source include/ must not be used
    module.addIncludePath(libs.sdl2.path(b, "include/SDL2"));
    module.addIncludePath(libs.sdl2.path(b, if (optimize == .debug) "include-config-debug/SDL2" else "include-config-release/SDL2"));
    module.addIncludePath(webp_include);
    module.addIncludePath(abseil_include);
    // tiny_obj_loader.h, for plugin/obj_decoder
    module.addIncludePath(b.dependency("tinyobjloader", .{ .target = module.resolved_target.?, .optimize = optimize }).path(""));
    for ([_][]const u8{
        "filament/include",
        "filament/backend/include",
        "libs/filabridge/include",
        "libs/utils/include",
        "libs/math/include",
        "libs/image/include",
        "libs/ktxreader/include",
    }) |dir| module.addIncludePath(filament.path(b, dir));

    module.linkLibrary(mujoco_lib);

    // Filament, SDL2, libwebp and abseil, as built with CMake
    for ([_][]const u8{
        "filament/libfilament.a",
        "filament/backend/libbackend.a",
        "libs/bluegl/libbluegl.a",
        "libs/filabridge/libfilabridge.a",
        "libs/filaflat/libfilaflat.a",
        "libs/image/libimage.a",
        "libs/ktxreader/libktxreader.a",
        "libs/utils/libutils.a",
        "third_party/basisu/tnt/libbasis_transcoder.a",
        "third_party/zstd/tnt/libzstd.a",
    }) |lib| module.addObjectFile(libs.filament.path(b, lib));
    module.addObjectFile(libs.sdl2.path(b, if (optimize == .debug) "libSDL2d.a" else "libSDL2.a"));
    module.addObjectFile(libs.webp.path(b, "libwebp.a"));
    module.addObjectFile(libs.webp.path(b, "libsharpyuv.a"));
    if (!wrapper) module.addObjectFile(libs.abseil.path(b, "libmerged.a"));
}

// MuJoCo Studio as a native executable on studio/main.cc. The fonts and
// materials go to <prefix>/bin/assets, where it looks for them.
fn addStudio(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.lang.Optimize,
    mujoco_lib: *std.Build.Step.Compile,
    libs: StudioLibs,
    tools: FilamentTools,
    plugin_files: []const []const u8,
) !void {
    const module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libcpp = true,
    });
    try addStudioSources(b, module, optimize, mujoco_lib, libs, false, plugin_files);

    const exe = b.addExecutable(.{ .name = "studio", .root_module = module });
    const install_exe = b.addInstallArtifact(exe, .{});
    b.getInstallStep().dependOn(&install_exe.step);

    // MuJoCo's materials (src/render/filament/assets/*.mat) compiled with matc, with
    // the flags of src/render/filament/CMakeLists.txt, and the fonts, next to the binary
    const assets_step = b.step("studio-assets", "Install the fonts and materials studio loads at runtime");
    const assets_dir = "src/render/filament/assets";
    const io = b.graph.io;
    var dir = try mujoco_dep.openDir(io, assets_dir, .{ .iterate = true });
    defer dir.close(io);
    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.eql(u8, entry.name, "ibl.ktx")) {
            const install = b.addInstallFileWithDir(mujoco_dep.path(b.fmt("{s}/ibl.ktx", .{assets_dir})), .{ .custom = "bin/assets" }, "ibl.ktx");
            assets_step.dependOn(&install.step);
            continue;
        }
        if (!std.mem.endsWith(u8, entry.name, ".mat")) continue;

        const matc = std.Build.Step.Run.create(b, b.fmt("matc {s}", .{entry.name}));
        matc.addFileArg(tools.matc);
        matc.addArgs(&.{ "--platform=all", "--api=vulkan", "--api=opengl", "--variant-filter", "skinning", "--optimize-size" });
        matc.addArg("--output");
        const out = matc.addOutputFileArg(b.fmt("{s}.filamat", .{std.fs.path.stem(entry.name)}));
        matc.addFileArg(mujoco_dep.path(b.fmt("{s}/{s}", .{ assets_dir, entry.name })));

        const install = b.addInstallFileWithDir(out, .{ .custom = "bin/assets" }, b.fmt("{s}.filamat", .{std.fs.path.stem(entry.name)}));
        assets_step.dependOn(&install.step);
    }
    for ([_]struct { []const u8, []const u8, []const u8, []const u8 }{
        .{ "atkinson_next", "atkinson-hyperlegible-next-5d633f80fc654ef5fffa7cfc257528685158dcef", "fonts/variable/AtkinsonHyperlegibleNext[wght].ttf", "AtkinsonHyperlegibleNext[wght].ttf" },
        .{ "font_awesome", "Font-Awesome-a8386aae19e200ddb0f6845b5feeee5eb7013687", "fonts/fontawesome-webfont.ttf", "fontawesome-webfont.ttf" },
        .{ "atkinson_mono", "atkinson-hyperlegible-next-mono-154d50362016cc3e873eb21d242cd0772384c8f9", "fonts/ttf/AtkinsonHyperlegibleMono-Regular.ttf", "AtkinsonHyperlegibleMono-Regular.ttf" },
    }) |font| {
        const font_dep = b.lazyDependency(font[0], .{}) orelse return;
        const install = b.addInstallFileWithDir(font_dep.path(font[2]), .{ .custom = "bin/assets" }, font[3]);
        assets_step.dependOn(&install.step);
    }
    b.getInstallStep().dependOn(assets_step);

    // run the installed binary, which finds assets/ next to itself
    const run = std.Build.Step.Run.create(b, "run studio");
    run.addFileArg(.{ .relative = .{ .base = .install_bin, .sub_path = "studio" } });
    run.addPassthruArgs();
    run.step.dependOn(&install_exe.step);
    run.step.dependOn(assets_step);
    b.step("run-studio", "Run MuJoCo Studio").dependOn(&run.step);
}

pub fn build(b: *std.Build) !void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    // -Dsuppress-nonnull: skip the nonnull-attribute undefined-behaviour check
    // (Debug, ReleaseSafe) for MuJoCo's own code (engine, simulate, plugins). It
    // only fires on zero-length memcpy/memset with a null pointer, harmless in
    // practice, but it stops every run before the other checks can report
    // anything. Found at:
    //   simulate/simulate.cc:2343             memcpy of eq_active_ (empty vector), neq = 0
    //   src/engine/engine_core_smooth.c:1641  mju_zero(efc_force, nefc), nefc = 0
    //   src/engine/engine_io.c:896            mju_copyInt(remaining, ...) (empty vector), nv = 0
    const suppress_nonnull = b.option(bool, "suppress-nonnull", "Skip the nonnull-attribute check for MuJoCo's code (harmless zero-length memcpy/memset findings)") orelse false;
    const ub_flags: []const []const u8 = if (suppress_nonnull) &.{"-fno-sanitize=nonnull-attribute"} else &.{};

    mujoco_dep = .{ .dep = b.dependency("mujoco", .{}) };

    // Create the main MuJoCo library
    const lib = b.addLibrary(.{
        .name = "lib_mujoco",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        }),
    });

    // GLFW (raw C source; libX11, libwayland-client etc. are dlopen'd at runtime).
    // -Dglfw-platform: which window system to compile in. `auto` (default) takes
    // the session this build runs in (XDG_SESSION_TYPE), `both` builds the two
    // and GLFW picks at startup, Wayland first.
    const GlfwPlatform = enum { auto, x11, wayland, both };
    const glfw_platform = b.option(GlfwPlatform, "glfw-platform", "Window system GLFW is built for: auto, x11, wayland or both") orelse .auto;
    const session_wayland = if (b.graph.environ_map.get("XDG_SESSION_TYPE")) |session| std.mem.eql(u8, session, "wayland") else false;
    const use_wayland = switch (glfw_platform) {
        .auto => session_wayland,
        .x11 => false,
        .wayland, .both => true,
    };
    const use_x11 = switch (glfw_platform) {
        .auto => !session_wayland,
        .x11, .both => true,
        .wayland => false,
    };
    const glfw_flags = withFlags(b, &.{ "-std=c99", "-D_GNU_SOURCE", "-DHAVE_MEMFD_CREATE" }, if (use_x11 and use_wayland)
        &.{ "-D_GLFW_X11", "-D_GLFW_WAYLAND" }
    else if (use_x11)
        &.{"-D_GLFW_X11"}
    else
        &.{"-D_GLFW_WAYLAND"});
    const glfw_dep = b.dependency("glfw", .{});
    const glfw = b.addLibrary(.{
        .name = "glfw",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
        }),
    });
    glfw.root_module.addCSourceFiles(.{
        .root = glfw_dep.path("src"),
        .files = &.{
            "platform.c",
            "monitor.c",
            "init.c",
            "vulkan.c",
            "input.c",
            "context.c",
            "window.c",
            "osmesa_context.c",
            "egl_context.c",
            "null_init.c",
            "null_monitor.c",
            "null_window.c",
            "null_joystick.c",
            "posix_time.c",
            "posix_thread.c",
            "posix_module.c",
            "posix_poll.c",
            "xkb_unicode.c",
            "linux_joystick.c",
        },
        .flags = glfw_flags,
    });
    glfw.root_module.addIncludePath(glfw_dep.path("include"));
    if (use_x11) {
        glfw.root_module.addCSourceFiles(.{
            .root = glfw_dep.path("src"),
            .files = &.{ "x11_init.c", "x11_monitor.c", "x11_window.c", "glx_context.c" },
            .flags = glfw_flags,
        });
    }

    // Wayland protocol headers, generated by wayland-scanner from GLFW's XML
    // files as its CMake does. libwayland-client, libxkbcommon and libdecor are
    // dlopen'd at runtime.
    if (use_wayland) {
        glfw.root_module.addCSourceFiles(.{
            .root = glfw_dep.path("src"),
            .files = &.{ "wl_init.c", "wl_monitor.c", "wl_window.c" },
            .flags = glfw_flags,
        });
        const wayland_protocols = b.addWriteFiles();
        for ([_][]const u8{
            "wayland",
            "viewporter",
            "xdg-shell",
            "idle-inhibit-unstable-v1",
            "pointer-constraints-unstable-v1",
            "relative-pointer-unstable-v1",
            "fractional-scale-v1",
            "xdg-activation-v1",
            "xdg-decoration-unstable-v1",
        }) |name| {
            for ([_][]const u8{ "client-header", "private-code" }) |kind| {
                const suffix = if (std.mem.eql(u8, kind, "client-header")) "-client-protocol.h" else "-client-protocol-code.h";
                const scan = b.addSystemCommand(&.{ "wayland-scanner", kind });
                scan.addFileArg(glfw_dep.path(b.fmt("deps/wayland/{s}.xml", .{name})));
                const out = scan.addOutputFileArg(b.fmt("{s}{s}", .{ name, suffix }));
                _ = wayland_protocols.addCopyFile(out, b.fmt("{s}{s}", .{ name, suffix }));
            }
        }
        glfw.root_module.addIncludePath(wayland_protocols.getDirectory());
    }
    lib.root_module.linkLibrary(glfw);

    // libccd (raw C source)
    const libccd_dep = b.dependency("libccd", .{
        .target = target,
        .optimize = optimize,
    });

    // qhull (raw C source)
    const qhull_dep = b.dependency("qhull", .{
        .target = target,
        .optimize = optimize,
    });

    // lodepng (raw C++ source)
    const lodepng_dep = b.dependency("lodepng", .{
        .target = target,
        .optimize = optimize,
    });

    // tinyxml2 (raw C++ source)
    const tinyxml2_dep = b.dependency("tinyxml2", .{
        .target = target,
        .optimize = optimize,
    });

    // tinyobjloader (raw C++ source)
    const tinyobjloader_dep = b.dependency("tinyobjloader", .{
        .target = target,
        .optimize = optimize,
    });

    // marchingcubecpp (header-only)
    const marchingcubecpp_dep = b.dependency("marchingcubecpp", .{
        .target = target,
        .optimize = optimize,
    });

    // trianglemeshdistance (header-only)
    const trianglemeshdistance_dep = b.dependency("trianglemeshdistance", .{
        .target = target,
        .optimize = optimize,
    });

    // === Generate CCD config.h ===
    const ccd_config_h = b.addWriteFile("ccd/config.h", "#define CCD_DOUBLE\n");
    lib.root_module.addIncludePath(ccd_config_h.getDirectory());

    // === Add MuJoCo C Source Files ===
    lib.root_module.addCSourceFiles(.{
        .root = mujoco_dep.path("."),
        .files = &.{
            "src/user/user_init.c",
            "src/ui/ui_main.c",
            "src/render/classic/render_util.c",
            "src/render/classic/render_gl3.c",
            "src/render/classic/render_gl2.c",
            "src/render/classic/render_context.c",
            "src/render/classic/glad/glad.c",
            "src/engine/engine_vis_visualize.c",
            "src/engine/engine_init.c",
            "src/engine/engine_vis_interact.c",
            "src/engine/engine_vis_init.c",
            "src/engine/engine_util_spatial.c",
            "src/engine/engine_util_sparse.c",
            "src/engine/engine_util_solve.c",
            "src/engine/engine_util_misc.c",
            "src/engine/engine_util_errmem.c",
            "src/engine/engine_util_blas.c",
            "src/engine/engine_support.c",
            "src/engine/engine_solver.c",
            "src/engine/engine_setconst.c",
            "src/engine/engine_sensor.c",
            "src/engine/engine_ray.c",
            "src/engine/engine_print.c",
            "src/engine/engine_passive.c",
            "src/engine/engine_name.c",
            "src/engine/engine_memory.c",
            "src/engine/engine_island.c",
            "src/engine/engine_io.c",
            "src/engine/engine_inverse.c",
            "src/engine/engine_forward.c",
            "src/engine/engine_derivative_fd.c",
            "src/engine/engine_derivative.c",
            "src/engine/engine_core_util.c",
            "src/engine/engine_core_smooth.c",
            "src/engine/engine_core_constraint.c",
            "src/engine/engine_collision_sdf.c",
            "src/engine/engine_collision_primitive.c",
            "src/engine/engine_collision_gjk.c",
            "src/engine/engine_collision_driver.c",
            "src/engine/engine_collision_convex.c",
            "src/engine/engine_collision_box.c",
            "src/engine/engine_collision_continuous.c",
            "src/engine/engine_collision_flex.c",
            "src/engine/engine_ipc.c",
            "src/engine/engine_metric.c",
            "src/engine/engine_callback.c",
            "src/engine/engine_sleep.c",
        },
        .flags = withFlags(b, &.{
            "-std=c11",
            "-DmjUSEPLATFORMSIMD",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        }, ub_flags),
    });

    // === Add libccd C source files ===
    // Files are in src/ directory at the dependency root
    lib.root_module.addCSourceFiles(.{
        .root = libccd_dep.path("src"),
        .files = &.{
            "support.c",
            "mpr.c",
            "vec3.c",
            "polytope.c",
            "ccd.c",
        },
        .flags = &.{
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add qhull C source files ===
    // Files are in src/libqhull_r/ directory
    lib.root_module.addCSourceFiles(.{
        .root = qhull_dep.path("src/libqhull_r"),
        .files = &.{
            "global_r.c",
            "stat_r.c",
            "geom2_r.c",
            "poly2_r.c",
            "merge_r.c",
            "libqhull_r.c",
            "geom_r.c",
            "poly_r.c",
            "qset_r.c",
            "mem_r.c",
            "random_r.c",
            "usermem_r.c",
            "userprintf_r.c",
            "io_r.c",
            "user_r.c",
            "rboxlib_r.c",
            "userprintf_rbox_r.c",
        },
        .flags = &.{
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add MuJoCo C++ Source Files ===
    lib.root_module.addCSourceFiles(.{
        .root = mujoco_dep.path("."),
        .files = &.{
            "src/xml/xml_util.cc",
            "src/xml/xml_urdf.cc",
            "src/xml/xml_numeric_format.cc",
            "src/xml/xml_native_writer.cc",
            "src/xml/xml_native_reader.cc",
            "src/xml/xml_base.cc",
            "src/xml/xml_api.cc",
            "src/xml/xml.cc",
            "src/user/user_vfs.cc",
            "src/user/user_util.cc",
            "src/user/user_resource.cc",
            "src/user/user_objects.cc",
            "src/user/user_model.cc",
            "src/user/user_mesh.cc",
            "src/user/user_flexcomp.cc",
            "src/user/user_composite.cc",
            "src/user/user_cache.cc",
            "src/user/user_threadpool.cc",
            "src/user/user_api.cc",
            "src/user/user_resolver.cc",
            "src/batch/batch.cc",
            "src/render/classic/glad/loader.cc",
            "src/engine/engine_thread.cc",
            "src/engine/engine_plugin.cc",
            "src/engine/engine_crossplatform.cc",
            "src/xml/xml_global.cc",
        },
        .flags = withFlags(b, &.{
            "-std=c++20",
            "-DmjUSEPLATFORMSIMD",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        }, ub_flags),
    });

    // === Add third-party C++ files from fetched dependencies ===
    
    // lodepng (lodepng.cpp is at root)
    lib.root_module.addCSourceFiles(.{
        .root = lodepng_dep.path(""),
        .files = &.{"lodepng.cpp"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // tinyxml2 (tinyxml2.cpp is at root)
    lib.root_module.addCSourceFiles(.{
        .root = tinyxml2_dep.path(""),
        .files = &.{"tinyxml2.cpp"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // tinyobjloader (tiny_obj_loader.cc is at root)
    lib.root_module.addCSourceFiles(.{
        .root = tinyobjloader_dep.path(""),
        .files = &.{"tiny_obj_loader.cc"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add Include Paths ===
    
    // Local MuJoCo includes
    lib.root_module.addIncludePath(mujoco_dep.path("include"));
    lib.root_module.addIncludePath(mujoco_dep.path("src"));

    // libccd includes
    // Headers are in src/ and src/ccd/
    lib.root_module.addIncludePath(libccd_dep.path("src"));
    lib.root_module.addIncludePath(libccd_dep.path("src/ccd"));

    // qhull includes
    // Headers are in src/ and src/libqhull_r/
    lib.root_module.addIncludePath(qhull_dep.path("src"));
    lib.root_module.addIncludePath(qhull_dep.path("src/libqhull_r"));

    // lodepng includes
    // lodepng.h is at root
    lib.root_module.addIncludePath(lodepng_dep.path(""));

    // tinyxml2 includes
    // tinyxml2.h is at root
    lib.root_module.addIncludePath(tinyxml2_dep.path(""));

    // tinyobjloader includes
    // tiny_obj_loader.h is at root
    lib.root_module.addIncludePath(tinyobjloader_dep.path(""));

    // marchingcubecpp includes (header-only)
    // MC.h is at root
    lib.root_module.addIncludePath(marchingcubecpp_dep.path(""));

    // trianglemeshdistance includes (header-only)
    // Headers are in TriangleMeshDistance/include/tmd/
    // But the source uses: #include <TriangleMeshDistance/include/tmd/TriangleMeshDistance.h>
    // So we need to add the root path
    lib.root_module.addIncludePath(trianglemeshdistance_dep.path(""));

    b.installArtifact(lib);

    // === Create and export the lib_z_mujoco Zig module ===
    const lib_mujoco_module = b.addModule("lib_z_mujoco", .{
        .root_source_file = b.path("mujoco/mujoco.zig"),
        .target = target,
        .optimize = optimize,
    });

    lib_mujoco_module.addIncludePath(mujoco_dep.path("include"));
    lib_mujoco_module.addIncludePath(glfw_dep.path("include"));

    // mujoco.h translated for Zig (@cImport is gone in 0.17)
    const translate_c = b.addTranslateC(.{
        .root_source_file = mujoco_dep.path("include/mujoco/mujoco.h"),
        .target = target,
        .optimize = optimize,
    });
    translate_c.addIncludePath(mujoco_dep.path("include"));
    lib_mujoco_module.addImport("c", translate_c.createModule());

    // GLFW bindings copied from zglfw
    const glfw_options = b.addOptions();
    glfw_options.addOption(bool, "enable_x11", use_x11);
    glfw_options.addOption(bool, "enable_wayland", use_wayland);
    const glfw_module = b.createModule(.{
        .root_source_file = b.path("mujoco/glfw/glfw.zig"),
        .target = target,
        .optimize = optimize,
    });
    glfw_module.addOptions("glfw_options", glfw_options);
    lib_mujoco_module.addImport("glfw", glfw_module);
    lib_mujoco_module.linkLibrary(lib);

    // --- Simulate ---
    const simulate_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libcpp = true,
    });
    const simulate_files = &.{
        "simulate/main.cc",
        "simulate/simulate.cc",
        "simulate/platform_ui_adapter.cc",
        "simulate/glfw_adapter.cc",
        "simulate/glfw_dispatch.cc",
    };
    // Plugins are compiled straight into simulate: a separate shared library
    // would get its own copy of the static libmujoco and register there.
    const plugin_files = &.{
        "plugin/actuator/pid.cc",
        "plugin/actuator/register.cc",
        "plugin/elasticity/cable.cc",
        "plugin/elasticity/elasticity.cc",
        "plugin/elasticity/register.cc",
        "plugin/sdf/sdf.cc",
        "plugin/sdf/bolt.cc",
        "plugin/sdf/bowl.cc",
        "plugin/sdf/gear.cc",
        "plugin/sdf/nut.cc",
        "plugin/sdf/torus.cc",
        "plugin/sdf/register.cc",
        "plugin/sensor/touch_grid.cc",
        "plugin/sensor/register.cc",
        "plugin/stl_decoder/stl_decoder.cc",
        "plugin/obj_decoder/obj_decoder.cc",
    };
    simulate_module.addCSourceFiles(.{ .root = mujoco_dep.path("."), .files = simulate_files, .flags = withFlags(b, &.{"-std=c++20"}, ub_flags) });
    simulate_module.addCSourceFiles(.{ .root = mujoco_dep.path("."), .files = plugin_files, .flags = withFlags(b, &.{"-std=c++20"}, ub_flags) });
    simulate_module.addIncludePath(mujoco_dep.path("."));
    simulate_module.addIncludePath(mujoco_dep.path("include"));
    simulate_module.addIncludePath(mujoco_dep.path("simulate"));
    simulate_module.addIncludePath(lodepng_dep.path(""));
    simulate_module.addIncludePath(tinyobjloader_dep.path(""));
    simulate_module.addIncludePath(glfw_dep.path("include"));
    simulate_module.linkLibrary(lib);
    simulate_module.linkLibrary(glfw);

    const simulate_exe = b.addExecutable(.{
        .name = "simulate",
        .root_module = simulate_module,
    });
    b.installArtifact(simulate_exe);

    const run_simulate = b.addRunArtifact(simulate_exe);
    run_simulate.addPassthruArgs();
    b.step("run-simulate", "Run MuJoCo simulate").dependOn(&run_simulate.step);

    lib_mujoco_module.link_libcpp = true;
    lib_mujoco_module.linkLibrary(glfw);

    // === Create and export the zsimulate Zig module (python/mujoco/viewer.py) ===
    // The unmodified simulate sources (without main.cc) plus the C layer in
    // zig/mujoco/simulate.cc, adapted from the Python binding.
    const ui_color = b.option(
        enum { default, orange, white, black },
        "ui-color",
        "Default UI color of the Zig viewer",
    ) orelse .orange;
    const build_options = b.addOptions();
    build_options.addOption(c_int, "ui_color", @intFromEnum(ui_color));

    const zsimulate_module = b.addModule("zsimulate", .{
        .root_source_file = b.path("src/zsimulate.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
        },
        .link_libcpp = true,
    });
    zsimulate_module.addOptions("build_options", build_options);
    zsimulate_module.addCSourceFiles(.{
        .root = mujoco_dep.path("."),
        .files = &.{
            "simulate/simulate.cc",
            "simulate/platform_ui_adapter.cc",
            "simulate/glfw_adapter.cc",
            "simulate/glfw_dispatch.cc",
        },
        .flags = withFlags(b, &.{"-std=c++20"}, ub_flags),
    });
    zsimulate_module.addCSourceFiles(.{ .files = &.{"mujoco/simulate.cc"}, .flags = withFlags(b, &.{"-std=c++20"}, ub_flags) });
    zsimulate_module.addCSourceFiles(.{ .root = mujoco_dep.path("."), .files = plugin_files, .flags = withFlags(b, &.{"-std=c++20"}, ub_flags) });
    zsimulate_module.addIncludePath(mujoco_dep.path("."));
    zsimulate_module.addIncludePath(mujoco_dep.path("include"));
    zsimulate_module.addIncludePath(mujoco_dep.path("simulate"));
    zsimulate_module.addIncludePath(lodepng_dep.path(""));
    zsimulate_module.addIncludePath(tinyobjloader_dep.path(""));
    zsimulate_module.addIncludePath(glfw_dep.path("include"));
    zsimulate_module.linkLibrary(lib);
    zsimulate_module.linkLibrary(glfw);

    // === Create and export the renderer and media Zig modules ===
    // renderer: offscreen rendering (python/mujoco/renderer.py)
    // media: PNG and ffmpeg video output (mediapy in the Python notebooks)
    const renderer_module = b.addModule("renderer", .{
        .root_source_file = b.path("src/renderer.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
        },
    });
    const media_module = b.addModule("media", .{
        .root_source_file = b.path("src/media.zig"),
        .target = target,
        .optimize = optimize,
    });

    // minimize: box-bounded nonlinear least squares (python/mujoco/minimize.py)
    const minimize_module = b.addModule("minimize", .{
        .root_source_file = b.path("src/minimize.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
        },
    });

    // rollout: batched multi-threaded rollouts (python/mujoco/rollout.cc)
    const rollout_module = b.addModule("rollout", .{
        .root_source_file = b.path("src/rollout.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
        },
    });

    // lqr: humanoid balancing setpoint and gain (python/LQR.ipynb)
    const lqr_module = b.addModule("lqr", .{
        .root_source_file = b.path("src/lqr.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
        },
    });

    // --- Studio ---
    const filament_tools = addFilamentTools(b);
    const filament_libs = if (filament_tools) |tools| addFilamentLibs(b, tools, optimize) else null;
    const sdl2_lib = addCmakeStatic(b, optimize, "sdl2", "sdl2-lib", "-DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_TEST=OFF -DSDL_TESTS=OFF", "SDL2-static", null);
    const webp_lib = addCmakeStatic(
        b,
        optimize,
        "libwebp",
        "webp-lib",
        "-DBUILD_SHARED_LIBS=OFF -DWEBP_BUILD_ANIM_UTILS=OFF -DWEBP_BUILD_CWEBP=OFF -DWEBP_BUILD_DWEBP=OFF " ++
            "-DWEBP_BUILD_GIF2WEBP=OFF -DWEBP_BUILD_IMG2WEBP=OFF -DWEBP_BUILD_VWEBP=OFF -DWEBP_BUILD_WEBPINFO=OFF " ++
            "-DWEBP_BUILD_WEBPMUX=OFF -DWEBP_BUILD_EXTRAS=OFF",
        "webp",
        null,
    );
    // abseil's flags, which studio/main.cc parses its arguments with
    const abseil_lib = addCmakeStatic(
        b,
        optimize,
        "abseil",
        "abseil-lib",
        "-DABSL_PROPAGATE_CXX_STD=ON -DABSL_BUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=20",
        "flags_parse",
        "libabsl_*.a",
    );
    var zstudio_module: ?*std.Build.Module = null;
    if (filament_tools != null and filament_libs != null and sdl2_lib != null and webp_lib != null and abseil_lib != null) {
        const studio_libs: StudioLibs = .{
            .filament = filament_libs.?,
            .sdl2 = sdl2_lib.?,
            .webp = webp_lib.?,
            .abseil = abseil_lib.?,
        };
        try addStudio(b, target, optimize, lib, studio_libs, filament_tools.?, plugin_files);

        // zstudio: studio behind the C layer in zig/mujoco/studio.cc, like zsimulate
        const module = b.addModule("zstudio", .{
            .root_source_file = b.path("src/zstudio.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
            },
            .link_libcpp = true,
        });
        try addStudioSources(b, module, optimize, lib, studio_libs, true, plugin_files);
        zstudio_module = module;
    }

    // --- Samples and examples ---
    // zig/sample: ports of sample/; zig/examples: ports of the Python notebooks
    // and tools. Built by default like MUJOCO_BUILD_EXAMPLES in CMake; the
    // run-<name> steps work either way.
    const build_samples = b.option(bool, "samples", "Build and install zig/sample") orelse true;
    const build_examples = b.option(bool, "examples", "Build and install zig/examples") orelse true;
    const program_dirs = [_]struct { []const u8, bool }{
        .{ "sample", build_samples },
        .{ "examples", build_examples },
    };

    // extra module imports per program, on top of lib_z_mujoco
    const Import = std.Build.Module.Import;
    const zstudio_imports = [_]Import{.{ .name = "zstudio", .module = zstudio_module orelse lib_mujoco_module }};
    const example_imports = [_]struct { []const u8, []const Import }{
        .{ "zviewer", &.{.{ .name = "zsimulate", .module = zsimulate_module }} },
        .{ "zstudio", &zstudio_imports },
        .{ "zlqr", &.{ .{ .name = "lqr", .module = lqr_module }, .{ .name = "renderer", .module = renderer_module }, .{ .name = "media", .module = media_module } } },
        .{ "zlqr_viewer", &.{ .{ .name = "lqr", .module = lqr_module }, .{ .name = "zsimulate", .module = zsimulate_module } } },
        .{ "zleast_squares", &.{.{ .name = "minimize", .module = minimize_module }} },
        .{ "zrollout", &.{ .{ .name = "rollout", .module = rollout_module }, .{ .name = "renderer", .module = renderer_module }, .{ .name = "media", .module = media_module } } },
    };

    // ThreadSanitizer for all code, C/C++ and Zig: finds data races, e.g. in
    // MuJoCo's thread pool, simulate's physics/render threads, the passive
    // viewer and the rollout workers. -Dsanitize-thread
    const sanitize_thread = b.option(bool, "sanitize-thread", "Build everything with ThreadSanitizer (data races)") orelse false;
    const tsan: ?bool = if (sanitize_thread) true else null;
    for ([_]*std.Build.Module{
        lib.root_module,    glfw.root_module,     lib_mujoco_module, simulate_module,
        zsimulate_module,   renderer_module,      media_module,      minimize_module,
        rollout_module,     lqr_module,
    }) |m| m.sanitize_thread = tsan;

    // --- Tests: zig build test (no display), test-display, or test-<name> ---
    // zig/test/<name>_test.zig, run from the repository root (they load model/).
    // Built in Debug by default, so all the C/C++ code they reach runs with
    // Zig's undefined-behaviour checks.
    const test_step = b.step("test", "Run the tests that need no display");
    const test_display_step = b.step("test-display", "Run the tests that need a display (renderer, viewer)");
    const tests = [_]struct { []const u8, []const Import, bool }{
        .{ "minimize", &.{.{ .name = "minimize", .module = minimize_module }}, false },
        .{ "rollout", &.{.{ .name = "rollout", .module = rollout_module }}, false },
        .{ "lqr", &.{.{ .name = "lqr", .module = lqr_module }}, false },
        .{ "media", &.{.{ .name = "media", .module = media_module }}, false },
        .{ "models", &.{}, false },
        .{ "xml_fuzz", &.{}, false },
        .{ "renderer", &.{.{ .name = "renderer", .module = renderer_module }}, true },
        .{ "viewer", &.{.{ .name = "zsimulate", .module = zsimulate_module }}, true },
    };
    for (tests) |t| {
        const test_module = b.createModule(.{
            .root_source_file = b.path(b.fmt("test/{s}_test.zig", .{t[0]})),
            .target = target,
            .optimize = optimize,
            .sanitize_thread = tsan,
            .imports = &.{
                .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
            },
            .link_libcpp = true,
        });
        for (t[1]) |imp| test_module.addImport(imp.name, imp.module);

        // the models in model/plugin need the plugins compiled in
        if (std.mem.eql(u8, t[0], "models")) {
            test_module.addCSourceFiles(.{ .root = mujoco_dep.path("."), .files = plugin_files, .flags = withFlags(b, &.{"-std=c++20"}, ub_flags) });
            test_module.addIncludePath(mujoco_dep.path("."));
            test_module.addIncludePath(mujoco_dep.path("include"));
            test_module.addIncludePath(tinyobjloader_dep.path(""));
        }

        const test_exe = b.addTest(.{ .name = b.fmt("{s}_test", .{t[0]}), .root_module = test_module });
        const run_test = b.addRunArtifact(test_exe);
        run_test.setCwd(mujoco_dep.path("."));
        // skip false races inside graphics drivers (see the file)
        if (sanitize_thread) run_test.setEnvironmentVariable("TSAN_OPTIONS", "suppressions=zig/test/tsan.supp");
        // the models sweep is kept out of `test` for now (it stopped at the
        // nonnull findings listed at the top); run it with `zig build test-models`
        if (t[2]) {
            test_display_step.dependOn(&run_test.step);
        } else if (!std.mem.eql(u8, t[0], "models")) {
            test_step.dependOn(&run_test.step);
        }
        b.step(b.fmt("test-{s}", .{t[0]}), b.fmt("Run zig/test/{s}_test.zig", .{t[0]})).dependOn(&run_test.step);
    }

    const io = b.graph.io;
    for (program_dirs) |program_dir| {
        const dir_path = program_dir[0];
        const install = program_dir[1];
        var dir = b.root.openDir(io, dir_path, .{ .iterate = true }) catch |err| {
            if (err == error.FileNotFound) continue;
            return err;
        };
        defer dir.close(io);

        var dir_iter = dir.iterate();
        while (try dir_iter.next(io)) |entry| {
            if (entry.kind != .file or !std.mem.endsWith(u8, entry.name, ".zig")) {
                continue;
            }

            const exe_name = std.fs.path.stem(entry.name);
            // needs the studio dependencies, not fetched on the first pass
            if (std.mem.eql(u8, exe_name, "zstudio") and zstudio_module == null) continue;
            const root_source_path = b.path(b.fmt("{s}/{s}", .{ dir_path, entry.name }));

            const example_module = b.createModule(.{
                .root_source_file = root_source_path,
                .target = target,
                .optimize = optimize,
                .sanitize_thread = tsan,
                .imports = &.{
                    .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
                },
                .link_libcpp = true,
            });
            example_module.addIncludePath(mujoco_dep.path("include"));

            // opt-in, e.g. importing zsimulate compiles and links all of simulate
            for (example_imports) |extra| {
                if (!std.mem.eql(u8, exe_name, extra[0])) continue;
                for (extra[1]) |imp| example_module.addImport(imp.name, imp.module);
            }

            const exe = b.addExecutable(.{
                .name = exe_name,
                .root_module = example_module,
            });
            if (install) b.installArtifact(exe);

            // zstudio looks for assets/ next to its binary, so it runs from the install dir
            const run_exe_step = if (std.mem.eql(u8, exe_name, "zstudio")) blk: {
                const run = std.Build.Step.Run.create(b, "run zstudio");
                run.addFileArg(.{ .relative = .{ .base = .install_bin, .sub_path = "zstudio" } });
                run.step.dependOn(&b.addInstallArtifact(exe, .{}).step);
                run.step.dependOn(&b.top_level_steps.get("studio-assets").?.step);
                break :blk run;
            } else b.addRunArtifact(exe);
            run_exe_step.addPassthruArgs();
            if (sanitize_thread) {
                run_exe_step.setCwd(mujoco_dep.path("."));
                run_exe_step.setEnvironmentVariable("TSAN_OPTIONS", "suppressions=zig/test/tsan.supp");
            }
            const run_step_name = b.fmt("run-{s}", .{exe_name});
            const run_step_desc = b.fmt("Run {s}/{s}", .{ dir_path, exe_name });
            const run_step = b.step(run_step_name, run_step_desc);
            run_step.dependOn(&run_exe_step.step);
        }
    }
}
