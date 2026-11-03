// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// C layer for zig/src/zstudio.zig: studio's launcher behind a plain C function,
// so that the C++ studio sources can be used from Zig without studio/main.cc
// (which parses its flags with abseil).

#include "studio/launcher.h"

extern "C" {

// Launch studio and block until its window is closed; returns the exit code.
// title, model_file and gfx_mode may be null or empty, for the defaults.
int mjstudio_launch(int argc, char** argv, int window_width, int window_height,
                    const char* title, const char* model_file,
                    const char* gfx_mode) {
  mujoco::studio::LauncherConfig config;
  config.window_width = window_width;
  config.window_height = window_height;
  if (title && *title) config.title = title;
  if (model_file) config.model_file = model_file;
  if (gfx_mode) config.gfx_mode = gfx_mode;
  return mujoco::studio::LaunchStudio(argc, argv, config);
}

}  // extern "C"
