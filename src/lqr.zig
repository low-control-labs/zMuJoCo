const std = @import("std");
const c = @import("lib_z_mujoco").c;

// LQR balancing of the humanoid on one leg, from python/LQR.ipynb: the setpoint
// via inverse dynamics, the cost matrices, the linearization and the Riccati
// solve. Used by zig/examples/zlqr.zig (video) and zlqr_viewer.zig (viewer).

// cost coefficients
pub const balance_cost: f64 = 1000; // balancing
pub const balance_joint_cost: f64 = 3; // joints required for balancing
pub const other_joint_cost: f64 = 0.3; // other joints

/// What differs between models: how the setpoint pose is made, which bodies the
/// balance cost uses and which joints count as balance joints. The defaults are
/// the humanoid of python/LQR.ipynb.
pub const Config = struct {
    /// Reset d to the setpoint pose: set d.qpos, anything else is reset after.
    pose: *const fn (m: [*c]c.mjModel, d: [*c]c.mjData) anyerror!void = keyframe1,
    /// Root body, its subtree CoM is held over the stance foot body.
    torso: [:0]const u8 = "torso",
    foot: [:0]const u8 = "foot_left",
    /// Whether a joint name is a balance joint (cheap in the cost, small noise).
    is_balance_joint: *const fn (name: []const u8) bool = humanoidBalanceJoint,
    /// Actuators are position servos (ctrl = target length) rather than motors.
    position_servos: bool = false,
};

fn keyframe1(m: [*c]c.mjModel, d: [*c]c.mjData) anyerror!void {
    if (m.*.nkey < 2) return error.ModelNeedsKeyframe1;
    c.mj_resetDataKeyframe(m, d, 1);
}

fn humanoidBalanceJoint(name: []const u8) bool {
    const abdomen = has(name, "abdomen") and !has(name, "z");
    const left_leg = has(name, "left") and
        (has(name, "hip") or has(name, "knee") or has(name, "ankle")) and !has(name, "z");
    return abdomen or left_leg;
}

pub const Lqr = struct {
    nq: usize,
    nv: usize,
    nu: usize,
    na: usize,
    ndx: usize, // size of the linearized state, 2 nv + na
    qpos0: []f64, // position setpoint
    ctrl0: []f64, // control setpoint
    k: []f64, // feedback gain, nu x ndx
    is_balance: []bool, // per dof: abdomen and left-leg joints used for balancing
    dx: []f64, // scratch for control()
    kdx: []f64,

    /// Compute the setpoint and the LQR gain for model m (keyframe 1 = standing
    /// on one leg, bodies "torso" and "foot_left"). d is used as scratch.
    /// Everything is allocated with `allocator` (an arena is convenient).
    pub fn init(allocator: std.mem.Allocator, m: [*c]c.mjModel, d: [*c]c.mjData) !Lqr {
        return initWith(allocator, m, d, .{});
    }

    /// Same as init, for another model: see Config.
    pub fn initWith(allocator: std.mem.Allocator, m: [*c]c.mjModel, d: [*c]c.mjData, cfg: Config) !Lqr {
        const nq: usize = @intCast(m.*.nq);
        const nv: usize = @intCast(m.*.nv);
        const nu: usize = @intCast(m.*.nu);
        const na: usize = @intCast(m.*.na);
        const ndx = 2 * nv + na;

        // setpoint pose, made once
        c.mj_resetData(m, d);
        try cfg.pose(m, d);
        const qpos_pose = try allocator.dupe(f64, d.*.qpos[0..nq]);

        //---------------------------- setpoint via inverse dynamics ----------------------------

        // find the height offset at which the vertical root force is smallest
        var best_offset: f64 = 0;
        var best_force = std.math.inf(f64);
        for (0..2001) |i| {
            const offset = -0.001 + 1e-6 * @as(f64, @floatFromInt(i));
            c.mj_resetData(m, d);
            @memcpy(d.*.qpos[0..nq], qpos_pose);
            c.mj_forward(m, d);
            @memset(d.*.qacc[0..nv], 0);
            d.*.qpos[2] += offset;
            c.mj_inverse(m, d);
            if (@abs(d.*.qfrc_inverse[2]) < best_force) {
                best_force = @abs(d.*.qfrc_inverse[2]);
                best_offset = offset;
            }
        }
        std.log.info("smallest vertical force found {d:.3}mm above the keyframe", .{best_offset * 1000});

        // position and force setpoints
        c.mj_resetData(m, d);
        @memcpy(d.*.qpos[0..nq], qpos_pose);
        c.mj_forward(m, d);
        @memset(d.*.qacc[0..nv], 0);
        d.*.qpos[2] += best_offset;
        const qpos0 = try allocator.dupe(f64, d.*.qpos[0..nq]);
        c.mj_inverse(m, d);
        const qfrc0 = try allocator.dupe(f64, d.*.qfrc_inverse[0..nv]);

        // control setpoint: ctrl0 = qfrc0 * pinv(moment) = (M M')^-1 M qfrc0
        const moment = try allocator.alloc(f64, nu * nv);
        c.mju_sparse2dense(moment.ptr, d.*.actuator_moment, @intCast(nu), @intCast(nv), d.*.moment_rownnz, d.*.moment_rowadr, d.*.moment_colind);
        const mmt = try allocator.alloc(f64, nu * nu);
        c.mju_mulMatMatT(mmt.ptr, moment.ptr, moment.ptr, @intCast(nu), @intCast(nv), @intCast(nu));
        const ctrl0 = try allocator.alloc(f64, nu);
        c.mju_mulMatVec(ctrl0.ptr, moment.ptr, qfrc0.ptr, @intCast(nu), @intCast(nv));
        try solve(mmt, nu, ctrl0, 1);

        // position servos: force = gain ctrl + bias0 + bias1 length, so the
        // target that gives the force found above is (f - bias0 - bias1 length) / gain
        if (cfg.position_servos) {
            for (0..nu) |u| {
                const gain = m.*.actuator_gainprm[u * c.mjNGAIN];
                const bias0 = m.*.actuator_biasprm[u * c.mjNBIAS];
                const bias1 = m.*.actuator_biasprm[u * c.mjNBIAS + 1];
                ctrl0[u] = (ctrl0[u] - bias0 - bias1 * d.*.actuator_length[u]) / gain;
            }
        }

        //---------------------------------- cost matrices --------------------------------------

        // Qbalance: penalize the horizontal distance between CoM and left foot
        c.mj_resetData(m, d);
        @memcpy(d.*.qpos[0..nq], qpos0);
        c.mj_forward(m, d);
        const torso = c.mj_name2id(m, c.mjOBJ_BODY, cfg.torso);
        const foot = c.mj_name2id(m, c.mjOBJ_BODY, cfg.foot);
        if (torso < 0 or foot < 0) return error.ModelNeedsTorsoAndFootBodies;
        const jac_com = try allocator.alloc(f64, 3 * nv);
        const jac_foot = try allocator.alloc(f64, 3 * nv);
        c.mj_jacSubtreeCom(m, d, jac_com.ptr, torso);
        c.mj_jacBodyCom(m, d, jac_foot.ptr, null, foot);
        c.mju_sub(jac_com.ptr, jac_com.ptr, jac_foot.ptr, @intCast(3 * nv));
        const qpos_cost = try allocator.alloc(f64, nv * nv);
        c.mju_mulMatTMat(qpos_cost.ptr, jac_com.ptr, jac_com.ptr, 3, @intCast(nv), @intCast(nv));
        c.mju_scl(qpos_cost.ptr, qpos_cost.ptr, balance_cost, @intCast(nv * nv));

        // balance dofs: abdomen and left leg joints, except the z rotations
        const is_balance = try allocator.alloc(bool, nv);
        @memset(is_balance, false);
        for (0..@intCast(m.*.njnt)) |j| {
            const name_ptr = c.mj_id2name(m, c.mjOBJ_JOINT, @intCast(j));
            if (name_ptr == null) continue;
            const name = std.mem.span(name_ptr);
            if (cfg.is_balance_joint(name)) is_balance[@intCast(m.*.jnt_dofadr[j])] = true;
        }

        // Qjoint on the diagonal: free joint not penalized, balance and other joints weighted
        for (6..nv) |i| {
            qpos_cost[i * nv + i] += if (is_balance[i]) balance_joint_cost else other_joint_cost;
        }

        // Q: position block only, no explicit velocity penalty; R = identity
        const q_mat = try allocator.alloc(f64, ndx * ndx);
        @memset(q_mat, 0);
        for (0..nv) |i| @memcpy(q_mat[i * ndx ..][0..nv], qpos_cost[i * nv ..][0..nv]);
        const r_mat = try allocator.alloc(f64, nu * nu);
        c.mju_eye(r_mat.ptr, @intCast(nu));

        //-------------------------------------- LQR --------------------------------------------

        // linearize the dynamics around the setpoint
        c.mj_resetData(m, d);
        @memcpy(d.*.ctrl[0..nu], ctrl0);
        @memcpy(d.*.qpos[0..nq], qpos0);
        const a_mat = try allocator.alloc(f64, ndx * ndx);
        const b_mat = try allocator.alloc(f64, ndx * nu);
        c.mjd_transitionFD(m, d, 1e-6, true, a_mat.ptr, b_mat.ptr, null, null);

        // solve the discrete Riccati equation, K = (R + B'PB)^-1 B'PA
        const p_mat = try solveDiscreteAre(allocator, a_mat, b_mat, q_mat, r_mat, ndx, nu);
        const pb = try allocator.alloc(f64, ndx * nu);
        c.mju_mulMatMat(pb.ptr, p_mat.ptr, b_mat.ptr, @intCast(ndx), @intCast(ndx), @intCast(nu));
        const s_mat = try allocator.alloc(f64, nu * nu);
        c.mju_mulMatTMat(s_mat.ptr, b_mat.ptr, pb.ptr, @intCast(ndx), @intCast(nu), @intCast(nu));
        c.mju_addTo(s_mat.ptr, r_mat.ptr, @intCast(nu * nu));
        const pa = try allocator.alloc(f64, ndx * ndx);
        c.mju_mulMatMat(pa.ptr, p_mat.ptr, a_mat.ptr, @intCast(ndx), @intCast(ndx), @intCast(ndx));
        const k_mat = try allocator.alloc(f64, nu * ndx);
        c.mju_mulMatTMat(k_mat.ptr, b_mat.ptr, pa.ptr, @intCast(ndx), @intCast(nu), @intCast(ndx));
        try solve(s_mat, nu, k_mat, ndx);

        const dx = try allocator.alloc(f64, ndx);
        @memset(dx, 0);
        return .{
            .nq = nq,
            .nv = nv,
            .nu = nu,
            .na = na,
            .ndx = ndx,
            .qpos0 = qpos0,
            .ctrl0 = ctrl0,
            .k = k_mat,
            .is_balance = is_balance,
            .dx = dx,
            .kdx = try allocator.alloc(f64, nu),
        };
    }

    /// Reset d to the setpoint: qpos0, zero velocity, ctrl0.
    pub fn setPose(self: *const Lqr, m: [*c]const c.mjModel, d: [*c]c.mjData) void {
        c.mj_resetData(m, d);
        @memcpy(d.*.qpos[0..self.nq], self.qpos0);
        @memcpy(d.*.ctrl[0..self.nu], self.ctrl0);
        c.mj_forward(m, d);
    }

    /// LQR control law: ctrl = ctrl0 - K dx, with dx = [qpos - qpos0, qvel, act].
    pub fn control(self: *Lqr, m: [*c]const c.mjModel, d: [*c]c.mjData) void {
        c.mj_differentiatePos(m, self.dx.ptr, 1, self.qpos0.ptr, d.*.qpos);
        @memcpy(self.dx[self.nv .. 2 * self.nv], d.*.qvel[0..self.nv]);
        if (self.na > 0) @memcpy(self.dx[2 * self.nv ..], d.*.act[0..self.na]);
        c.mju_mulMatVec(self.kdx.ptr, self.k.ptr, self.dx.ptr, @intCast(self.nu), @intCast(self.ndx));
        for (0..self.nu) |u| d.*.ctrl[u] = self.ctrl0[u] - self.kdx[u];
    }

    /// Whether actuator u drives a balance dof (for perturbation scaling).
    pub fn isBalanceActuator(self: *const Lqr, m: [*c]const c.mjModel, u: usize) bool {
        const joint: usize = @intCast(m.*.actuator_trnid[2 * u]);
        return self.is_balance[@intCast(m.*.jnt_dofadr[joint])];
    }
};

fn has(s: []const u8, part: []const u8) bool {
    return std.mem.find(u8, s, part) != null;
}

/// Solve the discrete algebraic Riccati equation
///   P = A'PA - A'PB (R + B'PB)^-1 B'PA + Q
/// with the structure-preserving doubling algorithm:
///   W = I + G H,  A <- A W^-1 A,  G <- G + A W^-1 G A',  H <- H + A' H W^-1 A
/// starting from G = B R^-1 B', H = Q; H converges to P.
pub fn solveDiscreteAre(allocator: std.mem.Allocator, a_in: []const f64, b: []const f64, q: []const f64, r: []const f64, n: usize, nu: usize) ![]f64 {
    const nn = n * n;
    const ni: c_int = @intCast(n);
    const ak = try allocator.dupe(f64, a_in);
    const h = try allocator.dupe(f64, q);

    // G = B R^-1 B'
    const rinv_bt = try allocator.alloc(f64, nu * n);
    c.mju_transpose(rinv_bt.ptr, b.ptr, ni, @intCast(nu));
    try solve(try allocator.dupe(f64, r), nu, rinv_bt, n);
    const g = try allocator.alloc(f64, nn);
    c.mju_mulMatMat(g.ptr, b.ptr, rinv_bt.ptr, ni, @intCast(nu), ni);

    const w = try allocator.alloc(f64, nn);
    const w_lu = try allocator.alloc(f64, nn);
    const x1 = try allocator.alloc(f64, nn);
    const x2 = try allocator.alloc(f64, nn);
    const t1 = try allocator.alloc(f64, nn);
    const t2 = try allocator.alloc(f64, nn);

    for (0..100) |iter| {
        // W = I + G H; X1 = W^-1 A, X2 = W^-1 G
        c.mju_mulMatMat(w.ptr, g.ptr, h.ptr, ni, ni, ni);
        for (0..n) |i| w[i * n + i] += 1;
        @memcpy(x1, ak);
        @memcpy(w_lu, w);
        try solve(w_lu, n, x1, n);
        @memcpy(x2, g);
        @memcpy(w_lu, w);
        try solve(w_lu, n, x2, n);

        // H += A' H X1
        c.mju_mulMatMat(t1.ptr, h.ptr, x1.ptr, ni, ni, ni);
        c.mju_mulMatTMat(t2.ptr, ak.ptr, t1.ptr, ni, ni, ni);
        c.mju_addTo(h.ptr, t2.ptr, @intCast(nn));
        const h_update = c.mju_norm(t2.ptr, @intCast(nn)) / c.mju_norm(h.ptr, @intCast(nn));

        // G += A X2 A'
        c.mju_mulMatMat(t1.ptr, ak.ptr, x2.ptr, ni, ni, ni);
        c.mju_mulMatMatT(t2.ptr, t1.ptr, ak.ptr, ni, ni, ni);
        c.mju_addTo(g.ptr, t2.ptr, @intCast(nn));

        // A = A X1
        c.mju_mulMatMat(t1.ptr, ak.ptr, x1.ptr, ni, ni, ni);
        @memcpy(ak, t1);

        // converged when the H update is negligible
        if (h_update < 1e-12) {
            std.log.info("Riccati equation solved in {d} doubling iterations", .{iter + 1});
            return h;
        }
    }
    return error.RiccatiDidNotConverge;
}

/// Solve A X = B in place for X, with A n x n (overwritten) and B n x k,
/// both row-major. Gaussian elimination with partial pivoting.
pub fn solve(a: []f64, n: usize, b: []f64, k: usize) !void {
    for (0..n) |col| {
        // pivot
        var pivot = col;
        for (col + 1..n) |row| {
            if (@abs(a[row * n + col]) > @abs(a[pivot * n + col])) pivot = row;
        }
        if (a[pivot * n + col] == 0) return error.SingularMatrix;
        if (pivot != col) {
            for (0..n) |j| std.mem.swap(f64, &a[col * n + j], &a[pivot * n + j]);
            for (0..k) |j| std.mem.swap(f64, &b[col * k + j], &b[pivot * k + j]);
        }

        // eliminate below
        for (col + 1..n) |row| {
            const f = a[row * n + col] / a[col * n + col];
            if (f == 0) continue;
            for (col..n) |j| a[row * n + j] -= f * a[col * n + j];
            for (0..k) |j| b[row * k + j] -= f * b[col * k + j];
        }
    }

    // back substitution
    var row = n;
    while (row > 0) {
        row -= 1;
        for (0..k) |j| {
            var sum = b[row * k + j];
            for (row + 1..n) |i| sum -= a[row * n + i] * b[i * k + j];
            b[row * k + j] = sum / a[row * n + row];
        }
    }
}
