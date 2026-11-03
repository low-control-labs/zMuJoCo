<h1>
  <a href="#"><img alt="zMuJoCo" src="zmujoco.png" width="100%"/></a>
</h1>

<p>
  </a>
  </a>
  <a href="https://github.com/low-control-labs/mujoco/blob/main/LICENSE" alt="License">
    <img src="https://img.shields.io/github/license/low-control-labs/mujoco">
  </a>
</p>

# MuJoCo Zig Bindings

Zig bindings for the MuJoCo physics engine.

This package provides a way to build and interact with MuJoCo using Zig direct C interop feature.

> [!IMPORTANT]
> _These bindings are still a WIP. 

## Installation & Usage

> [!NOTE]
> Run all the commands in this README from the root directory of this project.

- To compile everything run `zig build` 
  
  ```sh
  zig build -Doptimize=ReleaseFast # use -Doptimize=ReleaseFast until patching logic for the upstream mujoco is added. 
  # Zig builds with more flags by default.
  # To reproduce the issue in upstream mujoco use -fsanitize=undefined flag
  ```

- To compile & run the native `simulate` or `studio` binary

  ```sh
  # -Dsuppress-nonnull due to tiny issues with upstream .cc and zig being more strict.
  zig build run-simulate -Dsuppress-nonnull # Or run-studio
  ```

- To use `simulate` via zig use the `zsimulate` module or the example `zviewer` binary

  ```sh
  # Use -Doptimize=ReleaseFast in case you want to skip all checks -Dsuppress-nonnull works too
  zig build run-zviewer -Doptimize=ReleaseFast
  ```

- To use `studio` via zig use the `zstudio` module or the example `zstudio` binary

  ```sh
  # Use -Doptimize=ReleaseFast in case you want to skip all checks -Dsuppress-nonnull works too but debug builds are much slower than the C/C++ version, still needs tests, not sure about the cause
  zig build run-zstudio -Doptimize=ReleaseFast
  ```

- To run the `lqr` example inside the zig `zviewer`

> [!WARNING]

- This was originally built inside the upstream mujoco repository so edit the filepath to your mujoco/models directory or copy it here.

  ```sh
  # Use -Doptimize=ReleaseFast or -Dsuppress-nonnull
  zig build run-zlqr_viewer -Doptimize=ReleaseFast
  # To add noise or any args use -- after the flags
  zig build run-zlqr_viewer -Doptimize=ReleaseFast -- --noise
  ```


## Debugging & Tests

Tests live in `zig/test/<name>_test.zig` and are run from the top-level directory:

```sh
zig build test                        # tests that need no display
zig build test-display                # renderer and viewer tests (need a display)
zig build test-<name>                 # a single test, e.g. test-minimize
zig build test-models                 # sweep over model/ (not part of `test`)
```

Tests build in Debug by default, so Zig's undefined-behaviour checks also run
on the C/C++ code they reach.

### `-Dsuppress-nonnull`

MuJoCo's own code (engine, simulate, plugins) trips the `nonnull-attribute`
check on zero-length `memcpy`/`memset` calls with a null pointer. These are
harmless in practice, but the check aborts the run before the other checks can
report anything:

- `simulate/simulate.cc:2343`: `memcpy` of `eq_active_` (empty vector), `neq = 0`
- `src/engine/engine_core_smooth.c:1641`: `mju_zero(efc_force, nefc)`, `nefc = 0`
- `src/engine/engine_io.c:896`: `mju_copyInt(remaining, ...)` (empty vector), `nv = 0`

To get past them, skip that one check for MuJoCo's code:

```sh
zig build test -Dsuppress-nonnull
```

It is off by default, so a plain `zig build test` in Debug stops at the first
of these findings. Release builds don't have the undefined-behaviour checks
and are unaffected.

### `-Dsanitize-thread`

Builds everything (C/C++ and Zig) with ThreadSanitizer to find data races in
MuJoCo's thread pool, the simulate physics/render threads, the passive viewer
and the rollout workers:

```sh
zig build test -Dsanitize-thread
```

## Future Work

1. **Improve the developer experience.**

2. **Improve the documentation & testing**

3. **Enable .wasm builds from zig, waiting for the zig package to update to 0.17.0.**

