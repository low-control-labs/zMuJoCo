const std = @import("std");
const minimize = @import("minimize");
const testing = std.testing;

// Port of python/mujoco/minimize_test.py. As in Python, the solver's messages
// go to an in-memory output and are checked for the expected text.

fn rosenbrock(x: []const f64, r: []f64) void {
    r[0] = 1 - x[0];
    r[1] = 10 * (x[1] - x[0] * x[0]);
}

const Rosenbrock = struct {
    pub fn residual(_: *const Rosenbrock, x: []const f64, r: []f64) !void {
        rosenbrock(x, r);
    }
};

fn RosenbrockJac(comptime bad: bool) type {
    return struct {
        pub fn residual(_: *const @This(), x: []const f64, r: []f64) !void {
            rosenbrock(x, r);
        }
        pub fn jacobian(_: *const @This(), x: []const f64, _: []const f64, jac: []f64) !void {
            jac[0] = -1;
            jac[1] = 0;
            jac[2] = -20 * x[0];
            jac[3] = if (bad) 15 else 10;
        }
    };
}

const RosenbrockN = struct {
    n: usize,
    pub fn residual(self: *const RosenbrockN, x: []const f64, r: []f64) !void {
        for (0..self.n - 1) |i| {
            r[i] = 1 - x[i];
            r[self.n - 1 + i] = 10 * (x[i] - x[i + 1] * x[i + 1]);
        }
    }
};

// in-memory output, like io.StringIO() in the Python test
const Output = struct {
    aw: std.Io.Writer.Allocating,

    fn init() Output {
        return .{ .aw = .init(testing.allocator) };
    }
    fn deinit(o: *Output) void {
        o.aw.deinit();
    }
    fn writer(o: *Output) *std.Io.Writer {
        return &o.aw.writer;
    }
    fn expectContains(o: *Output, text: []const u8) !void {
        if (std.mem.find(u8, o.aw.written(), text) == null) {
            std.debug.print("expected output to contain '{s}', got:\n{s}\n", .{ text, o.aw.written() });
            return error.TestExpectedText;
        }
    }
};

fn expectOnes(x: []const f64) !void {
    // np.testing.assert_array_almost_equal: decimal=6
    for (x) |xi| try testing.expectApproxEqAbs(1.0, xi, 1.5e-6);
}

test "basic" {
    var out: Output = .init();
    defer out.deinit();
    var result = try minimize.leastSquares(testing.allocator, testing.io, &Rosenbrock{}, &.{ 0, 0 }, 2, .{ .output = out.writer() });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try testing.expectEqual(minimize.Status.g_tol, result.status);
    try out.expectContains("norm(gradient) < tol");
}

test "start at minimum" {
    var out: Output = .init();
    defer out.deinit();
    var result = try minimize.leastSquares(testing.allocator, testing.io, &Rosenbrock{}, &.{ 1, 1 }, 2, .{ .output = out.writer() });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try out.expectContains("norm(gradient) < tol");
    try out.expectContains("exact minimum found");
}

test "jacobian callback" {
    var out: Output = .init();
    defer out.deinit();
    var result = try minimize.leastSquares(testing.allocator, testing.io, &RosenbrockJac(false){}, &.{ 0, 0 }, 2, .{
        .output = out.writer(),
        .check_derivatives = true,
    });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try out.expectContains("norm(gradient) < tol");
    try out.expectContains("Jacobian matches");

    // bad Jacobian, caught by check_derivatives
    try testing.expectError(error.JacobianMismatch, minimize.leastSquares(testing.allocator, testing.io, &RosenbrockJac(true){}, &.{ 0, 0 }, 2, .{
        .output = out.writer(),
        .check_derivatives = true,
    }));
    try out.expectContains("Jacobian does not match");
}

// records the range of points at which the residual is evaluated
const Probe = struct {
    lo: *f64,
    hi: *f64,
    pub fn residual(self: *const Probe, x: []const f64, r: []f64) !void {
        self.lo.* = @min(self.lo.*, x[0]);
        self.hi.* = @max(self.hi.*, x[0]);
        r[0] = x[0];
    }
};

test "jacobianFd respects bounds" {
    const cases = [_][3]f64{
        .{ 10, 20, 10 },
        .{ 10, 20, 20 },
        .{ -20, -10, -20 },
        .{ -20, -10, -10 },
    };
    const eps = @sqrt(std.math.floatEps(f64));
    for (cases) |c| {
        var lo: f64 = std.math.inf(f64);
        var hi: f64 = -std.math.inf(f64);
        const problem: Probe = .{ .lo = &lo, .hi = &hi };
        var jac = [_]f64{0};
        _ = try minimize.jacobianFd(testing.allocator, &problem, &.{c[2]}, &.{c[2]}, eps, .{ .lower = &.{c[0]}, .upper = &.{c[1]} }, &jac);
        try testing.expect(lo >= c[0]);
        try testing.expect(hi <= c[1]);
    }
}

test "max iter" {
    const n = 20; // high-dimensional Rosenbrock
    const x0: [n]f64 = @splat(0);
    var out: Output = .init();
    defer out.deinit();

    // fail to reach the minimum after 20 iterations
    {
        var result = try minimize.leastSquares(testing.allocator, testing.io, &RosenbrockN{ .n = n }, &x0, 2 * (n - 1), .{
            .output = out.writer(),
            .max_iter = 20,
        });
        defer result.deinit(testing.allocator);
        try out.expectContains("maximum iterations");
    }

    // succeed after 100 iterations (default)
    var result = try minimize.leastSquares(testing.allocator, testing.io, &RosenbrockN{ .n = n }, &x0, 2 * (n - 1), .{ .output = out.writer() });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
}

test "bounds" {
    const Case = struct { lower: [2]f64, upper: [2]f64 };
    const cases = [_]Case{
        .{ .lower = .{ -2, -2 }, .upper = .{ 2, 2 } }, // in bounds
        .{ .lower = .{ -2, 2 }, .upper = .{ 0.5, 3 } }, // on lower
        .{ .lower = .{ -2, -2 }, .upper = .{ 0.5, 2 } }, // on upper
    };

    // in bounds finds the true minimum
    {
        var out: Output = .init();
        defer out.deinit();
        var result = try minimize.leastSquares(testing.allocator, testing.io, &Rosenbrock{}, &.{ 0, 0 }, 2, .{
            .bounds = .{ .lower = &cases[0].lower, .upper = &cases[0].upper },
            .output = out.writer(),
        });
        defer result.deinit(testing.allocator);
        try expectOnes(result.x);
        try out.expectContains("norm(gradient) < tol");
    }

    // different bounds conditions
    for (cases) |case| {
        var out: Output = .init();
        defer out.deinit();
        var result = try minimize.leastSquares(testing.allocator, testing.io, &Rosenbrock{}, &.{ 0, 0 }, 2, .{
            .bounds = .{ .lower = &case.lower, .upper = &case.upper },
            .output = out.writer(),
            .verbose = .fulliter,
        });
        defer result.deinit(testing.allocator);
        try out.expectContains("norm(gradient) < tol");

        // if x_i is on the boundary the gradient points out, otherwise it is 0
        const grad = result.trace.items[result.trace.items.len - 1].grad.?;
        for (result.x, 0..) |xi, i| {
            if (xi == case.lower[i]) {
                try testing.expect(grad[i] > 0);
            } else if (xi == case.upper[i]) {
                try testing.expect(grad[i] < 0);
            } else {
                // assertAlmostEqual(places=4)
                try testing.expectApproxEqAbs(0.0, grad[i], 0.5e-4);
            }
        }
    }
}

test "bad bounds" {
    const x0 = [_]f64{ 0, 0 };
    const bad = [_]minimize.Bounds{
        // wrong size
        .{ .lower = &.{ -2, 2, 0 }, .upper = &.{ 0.5, 3, 4 } },
        // not finite
        .{ .lower = &.{ -2, 2 }, .upper = &.{ 0.5, std.math.inf(f64) } },
        // lower > upper
        .{ .lower = &.{ -2, 2 }, .upper = &.{ -5, 3 } },
    };
    var out: Output = .init();
    defer out.deinit();
    for (bad) |bounds| {
        try testing.expectError(error.BoundsInvalid, minimize.leastSquares(testing.allocator, testing.io, &Rosenbrock{}, &x0, 2, .{
            .bounds = bounds,
            .output = out.writer(),
        }));
    }
}

test "iter callback" {
    const Counted = struct {
        out: *Output,
        pub fn residual(_: *const @This(), x: []const f64, r: []f64) !void {
            rosenbrock(x, r);
        }
        pub fn iterCallback(self: *@This(), trace: []const minimize.IterLog) void {
            self.out.writer().print("Hello iteration {d}!\n", .{trace.len}) catch {};
        }
    };
    var out: Output = .init();
    defer out.deinit();
    var problem: Counted = .{ .out = &out };
    var result = try minimize.leastSquares(testing.allocator, testing.io, &problem, &.{ 0, 0 }, 2, .{ .output = out.writer() });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try out.expectContains("Hello iteration 3!");
}

//-------------------------------------- norms --------------------------------------------------

fn dot(a: []const f64, b: []const f64) f64 {
    var s: f64 = 0;
    for (a, b) |x, y| s += x * y;
    return s;
}

// hess = proj' * y_rr * proj, with y_rr nr x nr and proj nr x nx (row-major)
fn sandwich(proj: []const f64, y_rr: []const f64, nr: usize, nx: usize, hess: []f64) void {
    for (0..nx) |a| {
        for (0..nx) |b| {
            var s: f64 = 0;
            for (0..nr) |i| {
                for (0..nr) |k| s += proj[i * nx + a] * y_rr[i * nr + k] * proj[k * nx + b];
            }
            hess[a * nx + b] = s;
        }
    }
}

const Variant = enum { good, bad_grad, bad_hess, asym_hess, neg_hess };

// smooth-L2 norm with smoothing radius p, and the test's erroneous variants
fn SmoothL2(comptime variant: Variant) type {
    return struct {
        const p = 0.01;
        const max_nr = 8;

        pub fn residual(_: *const @This(), x: []const f64, r: []f64) !void {
            rosenbrock(x, r);
        }

        pub fn normValue(_: *const @This(), r: []const f64) f64 {
            return @sqrt(dot(r, r) + p * p) - p;
        }

        pub fn normGradHess(_: *const @This(), r: []const f64, proj: []const f64, nx: usize, grad: []f64, hess: []f64) void {
            const nr = r.len;
            const s = @sqrt(dot(r, r) + p * p);
            var y_r: [max_nr]f64 = undefined;
            for (0..nr) |i| y_r[i] = r[i] / s;

            // grad = proj' y_r (bad_grad: y_r + 0.001)
            for (0..nx) |a| {
                var g: f64 = 0;
                for (0..nr) |i| g += proj[i * nx + a] * (y_r[i] + if (variant == .bad_grad) 0.001 else 0);
                grad[a] = g;
            }

            // y_rr = (I - y_r y_r') / s, with the variants' errors
            var y_rr: [max_nr * max_nr]f64 = undefined;
            for (0..nr) |i| {
                for (0..nr) |k| {
                    const eye: f64 = if (i == k) (if (variant == .bad_hess) 1.001 else 1) else 0;
                    const yi = y_r[i] + if (variant == .asym_hess) 0.0001 else 0;
                    var v = (eye - yi * y_r[k]) / s;
                    if (variant == .neg_hess) v = -v;
                    y_rr[i * nr + k] = v;
                }
            }
            sandwich(proj, y_rr[0 .. nr * nr], nr, nx, hess);
        }
    };
}

test "norm" {
    var out: Output = .init();
    defer out.deinit();
    var result = try minimize.leastSquares(testing.allocator, testing.io, &SmoothL2(.good){}, &.{ 0, 0 }, 2, .{
        .output = out.writer(),
        .check_derivatives = true,
    });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try out.expectContains("norm(dx) < tol");
    try out.expectContains("User-provided norm gradient matches");
    try out.expectContains("User-provided norm Hessian matches");

    const opts: minimize.Options = .{ .output = out.writer(), .check_derivatives = true };
    try testing.expectError(error.JacobianMismatch, minimize.leastSquares(testing.allocator, testing.io, &SmoothL2(.bad_grad){}, &.{ 0, 0 }, 2, opts));
    try out.expectContains("gradient does not match");
    try testing.expectError(error.JacobianMismatch, minimize.leastSquares(testing.allocator, testing.io, &SmoothL2(.bad_hess){}, &.{ 0, 0 }, 2, opts));
    try out.expectContains("Hessian does not match");
    try testing.expectError(error.NormHessianNotSymmetric, minimize.leastSquares(testing.allocator, testing.io, &SmoothL2(.asym_hess){}, &.{ 0, 0 }, 2, opts));
    try out.expectContains("not symmetric");
    try testing.expectError(error.NormHessianNotPositiveDefinite, minimize.leastSquares(testing.allocator, testing.io, &SmoothL2(.neg_hess){}, &.{ 0, 0 }, 2, opts));
    try out.expectContains("not positive definite");
}

// 'soft_l1' loss of scipy least_squares
const SoftL1 = struct {
    pub fn residual(_: *const SoftL1, x: []const f64, r: []f64) !void {
        rosenbrock(x, r);
    }

    pub fn normValue(_: *const SoftL1, r: []const f64) f64 {
        var v: f64 = 0;
        for (r) |ri| v += @sqrt(ri * ri + 1) - 1;
        return v;
    }

    pub fn normGradHess(_: *const SoftL1, r: []const f64, proj: []const f64, nx: usize, grad: []f64, hess: []f64) void {
        const nr = r.len;
        var y_r: [8]f64 = undefined;
        var y_rr: [64]f64 = @splat(0);
        for (0..nr) |i| {
            const s = @sqrt(r[i] * r[i] + 1);
            y_r[i] = r[i] / s;
            y_rr[i * nr + i] = (1 - y_r[i] * y_r[i]) / s;
        }
        for (0..nx) |a| {
            var g: f64 = 0;
            for (0..nr) |i| g += proj[i * nx + a] * y_r[i];
            grad[a] = g;
        }
        sandwich(proj, y_rr[0 .. nr * nr], nr, nx, hess);
    }
};

test "soft l1 norm" {
    var out: Output = .init();
    defer out.deinit();
    var result = try minimize.leastSquares(testing.allocator, testing.io, &SoftL1{}, &.{ 0, 0 }, 2, .{
        .output = out.writer(),
        .check_derivatives = true,
    });
    defer result.deinit(testing.allocator);
    try expectOnes(result.x);
    try out.expectContains("User-provided norm gradient matches");
    try out.expectContains("User-provided norm Hessian matches");
}
