const std = @import("std");
const c = @import("lib_z_mujoco").c;

// Batched open-loop rollouts, a port of python/mujoco/rollout.cc and rollout.py.
// Rolls out nbatch trajectories of nstep steps from initial states, optionally
// applying controls, and records the resulting states and sensor values.
// Threads: one per mjData passed, as in Python (a single mjData runs on the
// calling thread). Threads are started per call; there is no persistent pool.

pub const Options = struct {
    /// mjtState signature of the control vectors (default: ctrl only)
    control_spec: c_int = c.mjSTATE_CTRL,
    /// in:  nbatch x nv qacc_warmstart arrays
    initial_warmstart: ?[]const f64 = null,
    /// in:  nbatch x nstep x ncontrol open-loop controls
    control: ?[]const f64 = null,
    /// out: nbatch x nstep x nstate (mjSTATE_FULLPHYSICS)
    state: ?[]f64 = null,
    /// out: nbatch x nstep x nsensordata
    sensordata: ?[]f64 = null,
    /// rollouts per thread job; default max(1, nbatch / (10 * nthread))
    chunk_size: ?usize = null,
};

pub const Error = error{
    NoData,
    ModelCount,
    InitialStateSize,
    WarmstartSize,
    ControlSize,
    StateSize,
    SensordataSize,
    ChunkSize,
};

/// Roll out nbatch = initial_state.len / nstate trajectories.
/// models: one model for all rollouts, or one per rollout (same sizes).
/// datas:  scratch mjData, one per thread; contents on entry do not matter.
pub fn rollout(
    models: []const [*c]const c.mjModel,
    datas: []const [*c]c.mjData,
    nstep: usize,
    initial_state: []const f64,
    options: Options,
) !void {
    if (datas.len == 0) return Error.NoData;
    if (models.len == 0) return Error.ModelCount;
    const m0 = models[0];

    // sizes
    const nstate: usize = @intCast(c.mj_stateSize(m0, c.mjSTATE_FULLPHYSICS));
    const ncontrol: usize = @intCast(c.mj_stateSize(m0, options.control_spec));
    const nv: usize = @intCast(m0.*.nv);
    const nsensordata: usize = @intCast(m0.*.nsensordata);
    if (initial_state.len == 0 or initial_state.len % nstate != 0) return Error.InitialStateSize;
    const nbatch = initial_state.len / nstate;
    if (models.len != 1 and models.len != nbatch) return Error.ModelCount;

    // argument sizes
    if (options.initial_warmstart) |w| if (w.len != nbatch * nv) return Error.WarmstartSize;
    if (options.control) |u| if (u.len != nbatch * nstep * ncontrol) return Error.ControlSize;
    if (options.state) |s| if (s.len != nbatch * nstep * nstate) return Error.StateSize;
    if (options.sensordata) |s| if (s.len != nbatch * nstep * nsensordata) return Error.SensordataSize;

    const job: Job = .{
        .models = models,
        .nstep = nstep,
        .control_spec = options.control_spec,
        .nstate = nstate,
        .ncontrol = ncontrol,
        .initial_state = initial_state,
        .initial_warmstart = options.initial_warmstart,
        .control = options.control,
        .state = options.state,
        .sensordata = options.sensordata,
    };

    // single data: run on the calling thread
    const nthread = if (datas.len > 1) datas.len else 0;
    if (nthread == 0) {
        job.run(datas[0], 0, nbatch);
        return;
    }

    // threaded: workers take chunks of rollouts from a shared counter
    const chunk_size = options.chunk_size orelse @max(1, nbatch / (nthread * 10));
    if (chunk_size == 0) return Error.ChunkSize;
    var next = std.atomic.Value(usize).init(0);

    var threads: [64]std.Thread = undefined;
    const nworker = @min(nthread, threads.len);
    var started: usize = 0;
    defer for (threads[0..started]) |t| t.join();
    for (0..nworker) |i| {
        threads[i] = try std.Thread.spawn(.{}, Job.worker, .{ &job, datas[i], &next, nbatch, chunk_size });
        started += 1;
    }
}

const Job = struct {
    models: []const [*c]const c.mjModel,
    nstep: usize,
    control_spec: c_int,
    nstate: usize,
    ncontrol: usize,
    initial_state: []const f64,
    initial_warmstart: ?[]const f64,
    control: ?[]const f64,
    state: ?[]f64,
    sensordata: ?[]f64,

    fn worker(job: *const Job, d: [*c]c.mjData, next: *std.atomic.Value(usize), nbatch: usize, chunk_size: usize) void {
        while (true) {
            const start = next.fetchAdd(chunk_size, .monotonic);
            if (start >= nbatch) return;
            job.run(d, start, @min(start + chunk_size, nbatch));
        }
    }

    fn model(job: *const Job, r: usize) [*c]const c.mjModel {
        return if (job.models.len == 1) job.models[0] else job.models[r];
    }

    // roll out trajectories [start_roll, end_roll) using d, as _unsafe_rollout in rollout.cc
    fn run(job: *const Job, d: [*c]c.mjData, start_roll: usize, end_roll: usize) void {
        const m0 = job.models[0];
        const nv: usize = @intCast(m0.*.nv);
        const nu: usize = @intCast(m0.*.nu);
        const nbody: usize = @intCast(m0.*.nbody);
        const neq: usize = @intCast(m0.*.neq);
        const nsensordata: usize = @intCast(m0.*.nsensordata);
        const spec = job.control_spec;

        // clear user inputs if unspecified
        if (spec & c.mjSTATE_CTRL == 0) @memset(d.*.ctrl[0..nu], 0);
        if (spec & c.mjSTATE_QFRC_APPLIED == 0) @memset(d.*.qfrc_applied[0..nv], 0);
        if (spec & c.mjSTATE_XFRC_APPLIED == 0) @memset(d.*.xfrc_applied[0 .. 6 * nbody], 0);

        for (start_roll..end_roll) |r| {
            const m = job.model(r);

            // clear user inputs if unspecified
            if (spec & c.mjSTATE_MOCAP_POS == 0) {
                for (0..nbody) |i| {
                    const id = m.*.body_mocapid[i];
                    if (id >= 0) c.mju_copy3(d.*.mocap_pos + 3 * @as(usize, @intCast(id)), m.*.body_pos + 3 * i);
                }
            }
            if (spec & c.mjSTATE_MOCAP_QUAT == 0) {
                for (0..nbody) |i| {
                    const id = m.*.body_mocapid[i];
                    if (id >= 0) c.mju_copy4(d.*.mocap_quat + 4 * @as(usize, @intCast(id)), m.*.body_quat + 4 * i);
                }
            }
            if (spec & c.mjSTATE_EQ_ACTIVE == 0) {
                for (0..neq) |i| d.*.eq_active[i] = m.*.eq_active0[i];
            }

            // initial state and warmstart
            c.mj_setState(m, d, job.initial_state[r * job.nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
            if (job.initial_warmstart) |w| {
                @memcpy(d.*.qacc_warmstart[0..nv], w[r * nv ..][0..nv]);
            } else {
                @memset(d.*.qacc_warmstart[0..nv], 0);
            }

            // clear warning counters
            for (&d.*.warning) |*w| w.number = 0;

            // roll out trajectory
            var t: usize = 0;
            while (t < job.nstep) : (t += 1) {
                // on any warning, fill the remaining outputs with the current outputs
                const warned = for (d.*.warning) |w| {
                    if (w.number != 0) break true;
                } else false;
                if (warned) {
                    while (t < job.nstep) : (t += 1) job.record(m, d, r * job.nstep + t, nsensordata);
                    break;
                }

                const step = r * job.nstep + t;
                if (job.control) |u| c.mj_setState(m, d, u[step * job.ncontrol ..].ptr, spec);
                c.mj_step(m, d);
                job.record(m, d, step, nsensordata);
            }
        }
    }

    fn record(job: *const Job, m: [*c]const c.mjModel, d: [*c]c.mjData, step: usize, nsensordata: usize) void {
        if (job.state) |s| c.mj_getState(m, d, s[step * job.nstate ..].ptr, c.mjSTATE_FULLPHYSICS);
        if (job.sensordata) |s| @memcpy(s[step * nsensordata ..][0..nsensordata], d.*.sensordata[0..nsensordata]);
    }
};
