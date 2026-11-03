const std = @import("std");
const minimize = @import("minimize");

// Toy examples from python/least_squares.ipynb: Rosenbrock and Beale functions,
// with and without box bounds, and the n-dimensional Rosenbrock. Also shows an
// analytic Jacobian (checked against finite differences) and a batched residual.
// USAGE: zleast_squares

// Rosenbrock function, f = r'r with r = (1 - x, 10 (y - x^2)); minimum at (1, 1)
const Rosenbrock = struct {
    pub fn residual(_: *const Rosenbrock, x: []const f64, r: []f64) !void {
        r[0] = 1 - x[0];
        r[1] = 10 * (x[1] - x[0] * x[0]);
    }
};

// the same with an analytic Jacobian
const RosenbrockJac = struct {
    pub fn residual(_: *const RosenbrockJac, x: []const f64, r: []f64) !void {
        r[0] = 1 - x[0];
        r[1] = 10 * (x[1] - x[0] * x[0]);
    }
    pub fn jacobian(_: *const RosenbrockJac, x: []const f64, _: []const f64, jac: []f64) !void {
        jac[0] = -1;
        jac[1] = 0;
        jac[2] = -20 * x[0];
        jac[3] = 10;
    }
};

// Beale function; minimum at (3, 0.5)
const Beale = struct {
    pub fn residual(_: *const Beale, x: []const f64, r: []f64) !void {
        r[0] = 1.5 - x[0] + x[0] * x[1];
        r[1] = 2.25 - x[0] + x[0] * x[1] * x[1];
        r[2] = 2.625 - x[0] + x[0] * x[1] * x[1] * x[1];
    }
};

// n-dimensional Rosenbrock (as written in the notebook); minimum at (1, ..., 1)
const RosenbrockN = struct {
    n: usize,
    pub fn residual(self: *const RosenbrockN, x: []const f64, r: []f64) !void {
        for (0..self.n - 1) |i| {
            r[i] = 1 - x[i];
            r[self.n - 1 + i] = 10 * (x[i] - x[i + 1] * x[i + 1]);
        }
    }
};

// the same with a batched residual: k points in one call (k x n in, k x nr out),
// which is where several points could be evaluated in parallel
const RosenbrockNBatch = struct {
    n: usize,
    pub fn residual(self: *const RosenbrockNBatch, x: []const f64, r: []f64) !void {
        const single: RosenbrockN = .{ .n = self.n };
        try single.residual(x, r);
    }
    pub fn residualBatch(self: *const RosenbrockNBatch, xs: []const f64, rs: []f64, k: usize) !void {
        const nr = 2 * (self.n - 1);
        for (0..k) |j| try self.residual(xs[j * self.n ..][0..self.n], rs[j * nr ..][0..nr]);
    }
};

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;

    // Rosenbrock
    std.debug.print("\n== Rosenbrock\n", .{});
    {
        var result = try minimize.leastSquares(gpa, io, &Rosenbrock{}, &.{ 0, 0 }, 2, .{});
        defer result.deinit(gpa);
        std.debug.print("x = {any}\n", .{result.x});
    }

    // Rosenbrock, analytic Jacobian checked against finite differences
    std.debug.print("\n== Rosenbrock, analytic Jacobian\n", .{});
    {
        var result = try minimize.leastSquares(gpa, io, &RosenbrockJac{}, &.{ 0, 0 }, 2, .{ .check_derivatives = true });
        defer result.deinit(gpa);
        std.debug.print("x = {any}\n", .{result.x});
    }

    // Beale
    std.debug.print("\n== Beale\n", .{});
    {
        var result = try minimize.leastSquares(gpa, io, &Beale{}, &.{ -3, -3 }, 3, .{});
        defer result.deinit(gpa);
        std.debug.print("x = {any}\n", .{result.x});
    }

    // Rosenbrock with bounds, 4 x 4 grid of initial points
    std.debug.print("\n== Rosenbrock with bounds\n", .{});
    try boundedGrid(gpa, io, &Rosenbrock{}, 2, .{ -0.3, -1 }, .{ 0.9, 1.9 }, 4, 0.1, true);

    // Beale with bounds, 5 x 5 grid of initial points
    std.debug.print("\n== Beale with bounds\n", .{});
    try boundedGrid(gpa, io, &Beale{}, 3, .{ -2, -1.3 }, .{ 1.5, 3 }, 5, 0.4, false);

    // n-dimensional Rosenbrock, single and batched residual
    const n = 20;
    const x0: [n]f64 = @splat(0);
    for ([_]bool{ false, true }) |batched| {
        std.debug.print("\n== {d}-dimensional Rosenbrock{s}\n", .{ n, if (batched) ", batched residual" else "" });
        var result = if (batched)
            try minimize.leastSquares(gpa, io, &RosenbrockNBatch{ .n = n }, &x0, 2 * (n - 1), .{})
        else
            try minimize.leastSquares(gpa, io, &RosenbrockN{ .n = n }, &x0, 2 * (n - 1), .{});
        defer result.deinit(gpa);

        // expected solution is a vector of 1's
        var err: f64 = 0;
        for (result.x) |xi| err += (xi - 1) * (xi - 1);
        std.debug.print("norm(x - 1) = {e:.2} {s}\n", .{ @sqrt(err), if (@sqrt(err) < 1e-8) "ok" else "FAILED" });
    }
}

// minimize from a grid of initial points inside the bounds, as in the notebook
fn boundedGrid(
    gpa: std.mem.Allocator,
    io: std.Io,
    problem: anytype,
    nr: usize,
    lower: [2]f64,
    upper: [2]f64,
    num: usize,
    inset: f64,
    flip_y: bool,
) !void {
    for (0..num) |i| {
        for (0..num) |j| {
            const iy = if (flip_y) num - i - 1 else i;
            const x0 = [2]f64{ linspace(lower[0] + inset, upper[0] - inset, num, j), linspace(lower[1] + inset, upper[1] - inset, num, iy) };
            var result = try minimize.leastSquares(gpa, io, problem, &x0, nr, .{
                .bounds = .{ .lower = &lower, .upper = &upper },
                .verbose = .silent,
            });
            defer result.deinit(gpa);
            std.debug.print("x0 = ({d:>6.3}, {d:>6.3})  ->  x = ({d:>6.3}, {d:>6.3})  in {d} iterations\n", .{
                x0[0], x0[1], result.x[0], result.x[1], result.trace.items.len - 1,
            });
        }
    }
}

// element k of np.linspace(a, b, num)
fn linspace(a: f64, b: f64, num: usize, k: usize) f64 {
    if (num == 1) return a;
    return a + (b - a) * @as(f64, @floatFromInt(k)) / @as(f64, @floatFromInt(num - 1));
}
