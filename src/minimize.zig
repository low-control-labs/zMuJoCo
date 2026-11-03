const std = @import("std");
const c = @import("lib_z_mujoco").c;

// Nonlinear least squares with box bounds, a port of python/mujoco/minimize.py.
// Gauss-Newton Hessian, Levenberg-Marquardt steps with an adaptive regularizer
// mu, box constraints through mju_boxQP and an Armijo sufficient-reduction rule.
//
// The problem is any pointer to a value with these methods (optional ones are
// detected at compile time):
//   residual(x: []const f64, r: []f64) !void                  required, one point
//   residualBatch(xs: []const f64, rs: []f64, k: usize) !void  optional, k points
//       (xs: k x n, rs: k x nr, row-major); used for finite differences, so the
//       n perturbed points can be evaluated in one call, e.g. in parallel
//   jacobian(x: []const f64, r: []const f64, jac: []f64) !void optional, nr x n
//   normValue(r: []const f64) f64                              optional norm,
//   normGradHess(r, proj, nx, grad, hess) void                 default quadratic
//       (projected gradient proj' dn/dr, Hessian proj' d2n/dr2 proj; proj: nr x nx)
//   iterCallback(trace: []const IterLog) void                  optional

pub const Verbosity = enum(u2) { silent, final, iter, fulliter };

pub const Status = enum {
    factorization_failed,
    no_improvement,
    max_iter,
    dx_tol,
    g_tol,

    pub fn message(status: Status) []const u8 {
        return switch (status) {
            .factorization_failed => "factorization failed.",
            .no_improvement => "insufficient reduction.",
            .max_iter => "maximum iterations reached.",
            .dx_tol => "norm(dx) < tol.",
            .g_tol => "norm(gradient) < tol.",
        };
    }
};

/// Log of a single iteration. residual, jacobian, grad and step are only
/// recorded with Verbosity.fulliter.
pub const IterLog = struct {
    candidate: []f64, // decision variable at the beginning of this iteration
    objective: f64, // objective at the candidate
    reduction: f64, // reduction of the objective during this iteration
    regularizer: f64, // regularizer used for this iteration
    residual: ?[]f64 = null,
    jacobian: ?[]f64 = null,
    grad: ?[]f64 = null,
    step: ?[]f64 = null,

    fn deinit(log: IterLog, gpa: std.mem.Allocator) void {
        gpa.free(log.candidate);
        if (log.residual) |s| gpa.free(s);
        if (log.jacobian) |s| gpa.free(s);
        if (log.grad) |s| gpa.free(s);
        if (log.step) |s| gpa.free(s);
    }
};

pub const Bounds = struct {
    lower: []const f64,
    upper: []const f64,
};

pub const Options = struct {
    bounds: ?Bounds = null,
    eps: f64 = @sqrt(std.math.floatEps(f64)), // finite-difference perturbation
    mu_min: f64 = 1e-6, // minimum value of the regularizer
    mu_max: f64 = 1e8, // maximum value of the regularizer
    mu_factor: f64 = std.math.pow(f64, 10, 0.1), // regularizer increase/decrease factor
    xtol: f64 = 1e-8, // termination tolerance on relative step size
    gtol: f64 = 1e-8, // termination tolerance on gradient norm
    max_iter: usize = 100,
    verbose: Verbosity = .iter,
    check_derivatives: bool = false, // compare user Jacobian and norm with finite differences
    output: ?*std.Io.Writer = null, // where messages go; null: stderr
};

// print a message to output, or to stderr when no output is given
fn say(output: ?*std.Io.Writer, comptime fmt: []const u8, args: anytype) void {
    if (output) |w| {
        w.print(fmt, args) catch {};
    } else {
        std.debug.print(fmt, args);
    }
}

pub const Result = struct {
    x: []f64, // best solution found
    trace: std.ArrayList(IterLog), // sequence of solution iterates
    status: Status, // why the solver terminated (printed by Python, returned here)

    pub fn deinit(result: *Result, gpa: std.mem.Allocator) void {
        for (result.trace.items) |log| log.deinit(gpa);
        result.trace.deinit(gpa);
        gpa.free(result.x);
    }
};

pub const Error = error{
    MuFactorInvalid,
    BoundsInvalid,
    X0NotFinite,
    JacobianMismatch,
    NormHessianNotSymmetric,
    NormHessianNotPositiveDefinite,
};

/// Minimize norm(residual(x)) subject to optional box bounds, starting at x0.
/// nr is the size of the residual.
pub fn leastSquares(
    gpa: std.mem.Allocator,
    io: std.Io,
    problem: anytype,
    x0: []const f64,
    nr: usize,
    options: Options,
) !Result {
    const P = Problem(@TypeOf(problem));
    const t_start_total = std.Io.Timestamp.now(io, .awake);
    const verbose = @intFromEnum(options.verbose);
    const output = options.output;
    const armijo_c1 = 1e-2; // constant for Armijo's sufficient-reduction rule

    if (options.mu_factor <= 1) return Error.MuFactorInvalid;

    // locals
    const n = x0.len;
    const ni: c_int = @intCast(n);
    var status: Status = .max_iter;
    var mu: f64 = 0; // optimistically start with no regularization
    var n_reduc: u6 = 0; // number of sequential mu reductions
    var n_res: usize = 0;
    var n_jac: usize = 0;
    var t_res: f64 = 0;
    var t_jac: f64 = 0;

    var arena_state: std.heap.ArenaAllocator = .init(gpa);
    defer arena_state.deinit();
    const arena = arena_state.allocator();
    const x = try gpa.dupe(f64, x0);
    errdefer gpa.free(x);
    const xnew = try arena.alloc(f64, n);
    const dx = try arena.alloc(f64, n);
    @memset(dx, 0);
    var r = try arena.alloc(f64, nr);
    var rnew = try arena.alloc(f64, nr);
    const jac = try arena.alloc(f64, nr * n);
    const grad = try arena.alloc(f64, n);
    const hess = try arena.alloc(f64, n * n);
    const hess_reg = try arena.alloc(f64, n * n);
    const scratch = try arena.alloc(f64, n * (n + 7));
    const tmp = try arena.alloc(f64, n);
    const dlower = try arena.alloc(f64, n);
    const dupper = try arena.alloc(f64, n);

    var trace: std.ArrayList(IterLog) = .empty;
    errdefer {
        for (trace.items) |log| log.deinit(gpa);
        trace.deinit(gpa);
    }

    // check bounds, clip x0
    if (options.bounds) |b| {
        if (b.lower.len != n or b.upper.len != n) return Error.BoundsInvalid;
        for (b.lower, b.upper) |lo, hi| {
            if (!std.math.isFinite(lo) or !std.math.isFinite(hi) or !(lo < hi)) return Error.BoundsInvalid;
        }
        for (x, b.lower, b.upper) |*xi, lo, hi| xi.* = std.math.clamp(xi.*, lo, hi);
    }
    for (x) |xi| if (!std.math.isFinite(xi)) return Error.X0NotFinite;

    // initial residual
    var t_start = std.Io.Timestamp.now(io, .awake);
    try problem.residual(x, r);
    t_res += secondsSince(io, t_start);
    n_res += 1;

    // minimize
    var i: usize = 0;
    while (i < options.max_iter) : (i += 1) {
        if (status != .max_iter) break;

        // objective
        const y = P.normValue(problem, r);

        // Jacobian
        t_start = std.Io.Timestamp.now(io, .awake);
        if (P.has_jacobian) {
            try problem.jacobian(x, r, jac);
            t_jac += secondsSince(io, t_start);
            n_jac += 1;
            if (i == 0 and options.check_derivatives) {
                n_res += try checkJacobian(arena, problem, x, r, jac, options.eps, options.bounds, "Jacobian", output);
            }
        } else {
            n_res += try jacobianFd(arena, problem, x, r, options.eps, options.bounds, jac);
            t_res += secondsSince(io, t_start);
        }

        // user-provided norm
        if (P.has_norm) {
            if (i == 0 and options.check_derivatives) try checkNorm(arena, problem, r, options.eps, output);
        }

        // gradient, Gauss-Newton Hessian
        P.normGradHess(problem, r, jac, n, grad, hess);

        // termination on the norm of the free (unclamped) gradient
        var g_sqr: f64 = 0;
        for (grad, 0..) |g, k| {
            if (options.bounds) |b| {
                const clamped_lower = x[k] == b.lower[k] and g > 0;
                const clamped_upper = x[k] == b.upper[k] and g < 0;
                if (clamped_lower or clamped_upper) continue;
            }
            g_sqr += g * g;
        }
        const g_norm = @sqrt(g_sqr);
        if (g_norm <= options.gtol) {
            status = .g_tol;
            if (g_norm == 0) say(output, "Zero gradient norm: exact minimum found?\n", .{});
            break;
        }

        // bounds relative to x
        if (options.bounds) |b| {
            for (0..n) |k| {
                dlower[k] = b.lower[k] - x[k];
                dupper[k] = b.upper[k] - x[k];
            }
        }
        const lower_ptr: [*c]const f64 = if (options.bounds != null) dlower.ptr else null;
        const upper_ptr: [*c]const f64 = if (options.bounds != null) dupper.ptr else null;

        // find a reduction satisfying Armijo's rule
        var armijo: f64 = -1;
        var reduction: f64 = 0;
        while (armijo < 0) {
            // increase mu until factorizable
            while (true) {
                @memcpy(hess_reg, hess);
                for (0..n) |k| hess_reg[k * n + k] += mu;
                const n_free = c.mju_boxQP(dx.ptr, scratch.ptr, null, hess_reg.ptr, grad.ptr, ni, lower_ptr, upper_ptr);
                if (n_free >= 0) break;
                if (mu >= options.mu_max) {
                    status = .factorization_failed;
                    break;
                }
                increaseMu(&mu, &n_reduc, options);
            }
            if (status != .max_iter) break;

            // new candidate, residual
            for (xnew, x, dx) |*xn, xi, d| xn.* = xi + d;
            t_start = std.Io.Timestamp.now(io, .awake);
            try problem.residual(xnew, rnew);
            t_res += secondsSince(io, t_start);
            n_res += 1;

            // new objective, reduction
            const ynew = P.normValue(problem, rnew);
            reduction = y - ynew;
            armijo = reduction + armijo_c1 * c.mju_dot(grad.ptr, dx.ptr, ni);

            if (armijo < 0) {
                if (mu >= options.mu_max) {
                    status = .no_improvement;
                    break;
                }
                increaseMu(&mu, &n_reduc, options);
            }
        }
        if (status != .max_iter) break;

        // reduction ratio
        c.mju_mulMatVec(tmp.ptr, hess.ptr, dx.ptr, ni, ni);
        const expected_reduction = -(c.mju_dot(grad.ptr, dx.ptr, ni) + 0.5 * c.mju_dot(dx.ptr, tmp.ptr, ni));
        var reduction_ratio: f64 = 0;
        if (expected_reduction <= 0) {
            if (verbose > 0) {
                if (expected_reduction == 0) {
                    say(output, "Zero expected reduction: exact minimum found?\n", .{});
                } else {
                    say(output, "Negative expected reduction: should not occur.\n", .{});
                }
            }
        } else {
            reduction_ratio = reduction / expected_reduction;
        }

        // iteration message
        const dx_norm = c.mju_norm(dx.ptr, ni);
        if (verbose >= @intFromEnum(Verbosity.iter)) {
            const logmu = if (mu > 0) @log10(mu) else -std.math.inf(f64);
            say(output, "iter: {d:<3}  y: {e:<10.3}  log10mu: {d:>4.1}  ratio: {d:<7.2}  dx: {e:<9.2}  reduction: {e:<9.2}\n", .{
                i, y, logmu, reduction_ratio, dx_norm, reduction,
            });
        }

        // log, iteration callback
        var log: IterLog = .{ .candidate = try gpa.dupe(f64, x), .objective = y, .reduction = reduction, .regularizer = mu };
        if (verbose >= @intFromEnum(Verbosity.fulliter)) {
            log.residual = try gpa.dupe(f64, r);
            log.jacobian = try gpa.dupe(f64, jac);
            log.grad = try gpa.dupe(f64, grad);
            log.step = try gpa.dupe(f64, dx);
        }
        trace.append(gpa, log) catch |err| {
            log.deinit(gpa);
            return err;
        };
        if (P.has_callback) problem.iterCallback(trace.items);

        // termination on step norm
        if (dx_norm < options.xtol * (options.xtol + c.mju_norm(x.ptr, ni))) {
            status = .dx_tol;
            break;
        }

        // modify the regularizer like in (Bazaraa, Sherali, and Shetty)
        if (reduction_ratio > 0.75) {
            decreaseMu(&mu, &n_reduc, options);
        } else if (reduction_ratio < 0.25) {
            increaseMu(&mu, &n_reduc, options);
        }

        // accept the proposal
        @memcpy(x, xnew);
        std.mem.swap([]f64, &r, &rnew);
    }
    // the iteration count as Python's loop variable reports it
    const iterations = if (i == options.max_iter and i > 0) i - 1 else i;

    // final log; unlike the others, its values are computed at the end point
    const yfinal = P.normValue(problem, r);
    var log: IterLog = .{ .candidate = try gpa.dupe(f64, x), .objective = yfinal, .reduction = 0, .regularizer = mu };
    if (verbose >= @intFromEnum(Verbosity.fulliter)) {
        t_start = std.Io.Timestamp.now(io, .awake);
        if (P.has_jacobian) {
            try problem.jacobian(x, r, jac);
            t_jac += secondsSince(io, t_start);
            n_jac += 1;
        } else {
            n_res += try jacobianFd(arena, problem, x, r, options.eps, options.bounds, jac);
            t_res += secondsSince(io, t_start);
        }
        P.normGradHess(problem, r, jac, n, grad, hess);
        log.residual = try gpa.dupe(f64, r);
        log.jacobian = try gpa.dupe(f64, jac);
        log.grad = try gpa.dupe(f64, grad);
    }
    trace.append(gpa, log) catch |err| {
        log.deinit(gpa);
        return err;
    };
    if (P.has_callback) problem.iterCallback(trace.items);

    // final diagnostics
    if (verbose > 0) {
        say(output, "Terminated after {d} iterations: {s} y: {e:.4}, Residual evals: {d}", .{
            iterations, status.message(), yfinal, n_res,
        });
        if (n_jac > 0) say(output, ", Jacobian evals: {d}", .{n_jac});
        say(output, "\n", .{});

        const time_total = secondsSince(io, t_start_total);
        if (time_total > 0) {
            const scale: f64 = if (time_total > 1) 1 else 1000;
            const units = if (time_total > 1) "s" else "ms";
            say(output, "total time {d:.1}{s} of which residual {d:.1}%", .{ scale * time_total, units, 100 * t_res / time_total });
            if (t_jac > 0) say(output, " Jacobian {d:.1}%", .{100 * t_jac / time_total});
            say(output, "\n", .{});
        }
    }

    return .{ .x = x, .trace = trace, .status = status };
}

/// Finite-difference Jacobian of the residual at x (nr x n, row-major).
/// Returns the number of residual evaluations (n).
pub fn jacobianFd(
    arena: std.mem.Allocator,
    problem: anytype,
    x: []const f64,
    r: []const f64,
    eps: f64,
    bounds: ?Bounds,
    jac: []f64,
) !usize {
    const P = Problem(@TypeOf(problem));
    const n = x.len;
    const nr = r.len;

    // perturbation per coordinate
    const eps_vec = try arena.alloc(f64, n);
    defer arena.free(eps_vec);
    for (eps_vec, x, 0..) |*e, xj, j| {
        e.* = eps;
        if (bounds) |b| {
            // step away from the nearer bound
            const mid = 0.5 * (b.upper[j] + b.lower[j]);
            if (xj > mid) e.* = -eps;
        }
        e.* *= @max(1.0, @abs(xj));
        e.* = (e.* + xj) - xj;
    }

    if (P.has_batch) {
        // all perturbed points in one call
        const xs = try arena.alloc(f64, n * n);
        defer arena.free(xs);
        const rs = try arena.alloc(f64, n * nr);
        defer arena.free(rs);
        for (0..n) |j| {
            @memcpy(xs[j * n ..][0..n], x);
            xs[j * n + j] += eps_vec[j];
        }
        try problem.residualBatch(xs, rs, n);
        for (0..n) |j| {
            for (0..nr) |k| jac[k * n + j] = (rs[j * nr + k] - r[k]) / eps_vec[j];
        }
    } else {
        // one perturbed point at a time
        const xh = try arena.dupe(f64, x);
        defer arena.free(xh);
        const rh = try arena.alloc(f64, nr);
        defer arena.free(rh);
        for (0..n) |j| {
            xh[j] = x[j] + eps_vec[j];
            try problem.residual(xh, rh);
            xh[j] = x[j];
            for (0..nr) |k| jac[k * n + j] = (rh[k] - r[k]) / eps_vec[j];
        }
    }
    return n;
}

/// Check a user-provided Jacobian against finite differences.
/// Returns the number of residual evaluations.
pub fn checkJacobian(
    arena: std.mem.Allocator,
    problem: anytype,
    x: []const f64,
    r: []const f64,
    jac: []const f64,
    eps: f64,
    bounds: ?Bounds,
    name: []const u8,
    output: ?*std.Io.Writer,
) !usize {
    const jac_fd = try arena.alloc(f64, jac.len);
    defer arena.free(jac_fd);
    const n_res = try jacobianFd(arena, problem, x, r, eps, bounds, jac_fd);
    try compareFd(jac, jac_fd, name, output);
    return n_res;
}

// relative comparison used by check_jacobian
fn compareFd(jac: []const f64, jac_fd: []const f64, name: []const u8, output: ?*std.Io.Writer) !void {
    var denom: f64 = 1e-8;
    for (jac, jac_fd) |a, b| denom += @abs(a) + @abs(b);
    for (jac, jac_fd) |a, b| {
        if (@abs(a - b) / denom > 1e-5) {
            say(output, "User-provided {s} does not match finite-differences to a relative tolerance of 1e-5.\n", .{name});
            return Error.JacobianMismatch;
        }
    }
    say(output, "User-provided {s} matches finite-differences.\n", .{name});
}

/// Check a user-provided norm: symmetric positive-definite Hessian, and
/// gradient and Hessian matching finite differences.
fn checkNorm(arena: std.mem.Allocator, problem: anytype, r: []const f64, eps: f64, output: ?*std.Io.Writer) !void {
    const nr = r.len;
    const nri: c_int = @intCast(nr);
    const eye = try arena.alloc(f64, nr * nr);
    c.mju_eye(eye.ptr, nri);
    const n_g = try arena.alloc(f64, nr);
    const n_h = try arena.alloc(f64, nr * nr);
    problem.normGradHess(r, eye, nr, n_g, n_h);

    // symmetric (numpy allclose tolerances)
    for (0..nr) |i| {
        for (0..nr) |j| {
            const a = n_h[i * nr + j];
            const b = n_h[j * nr + i];
            if (@abs(a - b) > 1e-8 + 1e-5 * @abs(b)) {
                say(output, "User-provided norm Hessian is not symmetric.\n", .{});
                return Error.NormHessianNotSymmetric;
            }
        }
    }

    // positive definite: Cholesky with mindiag 0 loses rank only on negative pivots
    const chol = try arena.dupe(f64, n_h);
    if (c.mju_cholFactor(chol.ptr, nri, 0) < nri) {
        say(output, "User-provided norm Hessian is not positive definite.\n", .{});
        return Error.NormHessianNotPositiveDefinite;
    }

    // finite-difference gradient and Hessian of the norm
    const g_fd = try arena.alloc(f64, nr);
    const h_fd = try arena.alloc(f64, nr * nr);
    const rh = try arena.dupe(f64, r);
    const gh = try arena.alloc(f64, nr);
    const hh = try arena.alloc(f64, nr * nr);
    const value = problem.normValue(r);
    for (0..nr) |j| {
        var e = eps * @max(1.0, @abs(r[j]));
        e = (e + r[j]) - r[j];
        rh[j] = r[j] + e;
        g_fd[j] = (problem.normValue(rh) - value) / e;
        problem.normGradHess(rh, eye, nr, gh, hh);
        for (0..nr) |k| h_fd[k * nr + j] = (gh[k] - n_g[k]) / e;
        rh[j] = r[j];
    }
    try compareFd(n_g, g_fd, "norm gradient", output);
    try compareFd(n_h, h_fd, "norm Hessian", output);
}

fn increaseMu(mu: *f64, n_reduc: *u6, options: Options) void {
    // increase mu carefully: always by mu_factor
    mu.* = @max(options.mu_min, options.mu_factor * mu.*);
    n_reduc.* = 0;
}

fn decreaseMu(mu: *f64, n_reduc: *u6, options: Options) void {
    // decrease mu aggressively: sequential decreases grow exponentially
    const dmu = std.math.pow(f64, 1 / options.mu_factor, @floatFromInt(@as(u64, 1) << n_reduc.*));
    mu.* = if (mu.* * dmu < options.mu_min) 0 else mu.* * dmu;
    n_reduc.* +|= 1;
}

fn secondsSince(io: std.Io, start: std.Io.Timestamp) f64 {
    const elapsed = start.durationTo(std.Io.Timestamp.now(io, .awake));
    return @as(f64, @floatFromInt(elapsed.nanoseconds)) / std.time.ns_per_s;
}

// compile-time view of the problem type: which optional methods it has, and
// the default quadratic norm
fn Problem(comptime Ptr: type) type {
    const T = @typeInfo(Ptr).pointer.child;
    return struct {
        const has_batch = @hasDecl(T, "residualBatch");
        const has_jacobian = @hasDecl(T, "jacobian");
        const has_norm = @hasDecl(T, "normValue") and @hasDecl(T, "normGradHess");
        const has_callback = @hasDecl(T, "iterCallback");

        fn normValue(problem: Ptr, r: []const f64) f64 {
            if (has_norm) return problem.normValue(r);
            return 0.5 * c.mju_dot(r.ptr, r.ptr, @intCast(r.len));
        }

        // projected gradient proj' dn/dr and Hessian proj' d2n/dr2 proj
        fn normGradHess(problem: Ptr, r: []const f64, proj: []const f64, nx: usize, grad: []f64, hess: []f64) void {
            if (has_norm) return problem.normGradHess(r, proj, nx, grad, hess);
            const nri: c_int = @intCast(r.len);
            const nxi: c_int = @intCast(nx);
            c.mju_mulMatTVec(grad.ptr, proj.ptr, r.ptr, nri, nxi); // proj' r
            c.mju_mulMatTMat(hess.ptr, proj.ptr, proj.ptr, nri, nxi, nxi); // proj' proj
        }
    };
}
