// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modified from python/mujoco/simulate.cc: the pybind11 layer is replaced by
// plain C functions on an opaque handle, so that the C++ Simulate class can be used

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>  // NOLINT(build/c++11)
#include <tuple>
#include <utility>
#include <vector>

#include <glfw_adapter.h>
#include <simulate.h>
#include <mujoco/mujoco.h>

namespace {
using UIAdapter = mujoco::GlfwAdapter;

// key callback: called with the GLFW key code on key-down events
typedef void (*mjsimKeyCallback)(int key, void* userdata);

template <typename Adapter>
class UIAdapterWithKeyCallback : public Adapter {
 public:
  template <typename... Args>
  UIAdapterWithKeyCallback(mjsimKeyCallback key_callback, void* userdata, Args&&... args)
      : Adapter(std::forward<Args>(args)...),
        key_callback_(key_callback),
        userdata_(userdata) {}

 protected:
  void OnKey(int key, int scancode, int act) override {
    Adapter::OnKey(key, scancode, act);
    if (this->IsKeyDownEvent(act) && key_callback_) {
      key_callback_(this->last_key_, userdata_);
    }
  }

 private:
  mjsimKeyCallback key_callback_ = nullptr;
  void* userdata_ = nullptr;
};
}  // namespace

extern "C" {

// overlay text: [font, gridpos, text1, text2] as in mjr_overlay
typedef struct {
  int font;
  int gridpos;
  const char* text1;
  const char* text2;
} mjsimText;

// figure drawn at a viewport of the window
typedef struct {
  mjrRect viewport;
  const mjvFigure* figure;
} mjsimFigure;

// RGB image (height x width x 3, uint8) drawn at a viewport of the window
typedef struct {
  mjrRect viewport;
  const unsigned char* rgb;
  int width;
  int height;
} mjsimImage;

// Opaque handle. The camera, options and perturbation are owned here, like
// the Python objects that SimulateWrapper keeps alive.
struct mjsim_ {
  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mujoco::Simulate* simulate = nullptr;  // null once destroyed

  // set after RenderLoop returns; outlives simulate, so exit() can wait on it
  std::atomic_int render_done = 0;
};
typedef struct mjsim_ mjsim;

}  // extern "C"

namespace {
// Waits for an atomic value to become the expected value. Returns true if the
// simulate object has been destroyed and the caller should return.
bool WaitForAtomic(mjsim* h, std::atomic_int& atomic, int expected) {
  while (h->simulate && atomic.load() != expected) {
    // The render loop never consumes the pending request once exitrequest is set.
    if (h->simulate->exitrequest.load()) return true;
    std::this_thread::yield();
  }
  return h->simulate == nullptr;
}
}  // namespace

extern "C" {

//------------------------------------- lifecycle -------------------------------------------------

// Create the viewer and its window. Must be called on the thread that will run
// mjsim_renderLoop. user_scn (nullable, owned by the caller) holds extra geoms
// to draw; key_callback (nullable) is called on key-down events.
mjsim* mjsim_create(mjvScene* user_scn, int is_passive,
                    mjsimKeyCallback key_callback, void* userdata) {
  mjsim* h = new mjsim;
  mjv_defaultCamera(&h->cam);
  mjv_defaultOption(&h->opt);
  mjv_defaultPerturb(&h->pert);
  h->simulate = new mujoco::Simulate(
      std::make_unique<UIAdapterWithKeyCallback<UIAdapter>>(key_callback, userdata),
      &h->cam, &h->opt, &h->pert, is_passive != 0);
  h->simulate->user_scn = user_scn;
  return h;
}

// Delete the simulate object (and its window). The handle stays valid.
void mjsim_destroy(mjsim* h) {
  if (h->simulate) {
    delete h->simulate;
    h->simulate = nullptr;
  }
}

// Free the handle, destroying the simulate object if still alive.
void mjsim_free(mjsim* h) {
  mjsim_destroy(h);
  delete h;
}

// Run the UI loop until the window is closed or exit is requested (blocking).
void mjsim_renderLoop(mjsim* h) {
  if (h->simulate) h->simulate->RenderLoop();
  h->render_done.store(1);
}

// Request exit and wait until the UI loop has finished.
void mjsim_exit(mjsim* h) {
  if (h->simulate) {
    int value = 0;
    h->simulate->exitrequest.compare_exchange_strong(value, 1);
  }
  while (!h->render_done.load()) {
    std::this_thread::yield();
  }
}

//-------------------------------------- loading --------------------------------------------------

void mjsim_loadMessage(mjsim* h, const char* displayed_filename) {
  if (h->simulate) h->simulate->LoadMessage(displayed_filename);
}

void mjsim_loadMessageClear(mjsim* h) {
  if (h->simulate) h->simulate->LoadMessageClear();
}

// Hand a model and data (owned by the caller) to the viewer; blocks until the
// UI loop has taken them over.
void mjsim_load(mjsim* h, mjModel* m, mjData* d, const char* displayed_filename) {
  if (h->simulate) h->simulate->Load(m, d, displayed_filename);
}

//----------------------------------- synchronization ---------------------------------------------

// Exchange state with the UI and update the visualization (locks internally).
void mjsim_sync(mjsim* h, int state_only) {
  if (h->simulate) h->simulate->Sync(state_only != 0);
}

void mjsim_lock(mjsim* h) {
  if (h->simulate) h->simulate->mtx.lock();
}

void mjsim_unlock(mjsim* h) {
  if (h->simulate) h->simulate->mtx.unlock();
}

void mjsim_addToHistory(mjsim* h) {
  if (h->simulate) h->simulate->AddToHistory();
}

//---------------------------------- user visualization -------------------------------------------

mjvCamera* mjsim_cam(mjsim* h) { return &h->cam; }
mjvOption* mjsim_opt(mjsim* h) { return &h->opt; }
mjvPerturb* mjsim_pert(mjsim* h) { return &h->pert; }

// viewport of the 3D view, i.e. the window without the UI panels
mjrRect mjsim_viewport(mjsim* h) {
  if (!h->simulate) return mjrRect{0, 0, 0, 0};
  return h->simulate->uistate.rect[3];
}

void mjsim_setFigures(mjsim* h, const mjsimFigure* figures, int n) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newfigurerequest, 0)) return;
  for (int i = 0; i < n; i++) {
    h->simulate->user_figures_new_.push_back(
        std::make_pair(figures[i].viewport, *figures[i].figure));
  }
  int value = 0;
  h->simulate->newfigurerequest.compare_exchange_strong(value, 1);
}

void mjsim_clearFigures(mjsim* h) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newfigurerequest, 0)) return;
  h->simulate->user_figures_new_.clear();
  int value = 0;
  h->simulate->newfigurerequest.compare_exchange_strong(value, 1);
}

void mjsim_setTexts(mjsim* h, const mjsimText* texts, int n) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newtextrequest, 0)) return;
  for (int i = 0; i < n; i++) {
    h->simulate->user_texts_new_.push_back(std::make_tuple(
        texts[i].font, texts[i].gridpos,
        std::string(texts[i].text1 ? texts[i].text1 : ""),
        std::string(texts[i].text2 ? texts[i].text2 : "")));
  }
  int value = 0;
  h->simulate->newtextrequest.compare_exchange_strong(value, 1);
}

void mjsim_clearTexts(mjsim* h) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newtextrequest, 0)) return;
  h->simulate->user_texts_new_.clear();
  int value = 0;
  h->simulate->newtextrequest.compare_exchange_strong(value, 1);
}

void mjsim_setImages(mjsim* h, const mjsimImage* images, int n) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newimagerequest, 0)) return;
  for (int i = 0; i < n; i++) {
    // copy the pixels, the caller is not required to keep them
    size_t size = static_cast<size_t>(images[i].height) * images[i].width * 3;
    std::unique_ptr<unsigned char[]> image_copy(new unsigned char[size]());
    std::memcpy(image_copy.get(), images[i].rgb, size);
    h->simulate->user_images_new_.push_back(
        std::make_tuple(images[i].viewport, std::move(image_copy)));
  }
  int value = 0;
  h->simulate->newimagerequest.compare_exchange_strong(value, 1);
}

void mjsim_clearImages(mjsim* h) {
  if (!h->simulate || WaitForAtomic(h, h->simulate->newimagerequest, 0)) return;
  h->simulate->user_images_new_.clear();
  int value = 0;
  h->simulate->newimagerequest.compare_exchange_strong(value, 1);
}

void mjsim_updateHField(mjsim* h, int hfieldid) {
  if (h->simulate) h->simulate->UpdateHField(hfieldid);
}

void mjsim_updateMesh(mjsim* h, int meshid) {
  if (h->simulate) h->simulate->UpdateMesh(meshid);
}

void mjsim_updateTexture(mjsim* h, int texid) {
  if (h->simulate) h->simulate->UpdateTexture(texid);
}

//---------------------------------- simulate state -----------------------------------------------
// (accessors used by the physics loop in viewer.py / viewer.zig)

int mjsim_isAlive(mjsim* h) { return h->simulate != nullptr; }

int mjsim_exitrequest(mjsim* h) {
  return h->simulate ? h->simulate->exitrequest.load() : 2;
}

int mjsim_run(mjsim* h) { return h->simulate ? h->simulate->run : 0; }

int mjsim_uiloadrequest(mjsim* h) {
  return h->simulate ? h->simulate->uiloadrequest.load() : 0;
}

void mjsim_uiloadrequestDecrement(mjsim* h) {
  if (h->simulate) h->simulate->uiloadrequest.fetch_sub(1);
}

int mjsim_droploadrequest(mjsim* h) {
  return h->simulate ? h->simulate->droploadrequest.load() : 0;
}

void mjsim_setDroploadrequest(mjsim* h, int droploadrequest) {
  if (h->simulate) h->simulate->droploadrequest.store(droploadrequest);
}

const char* mjsim_dropfilename(mjsim* h) {
  return h->simulate ? h->simulate->dropfilename : "";
}

const char* mjsim_filename(mjsim* h) {
  return h->simulate ? h->simulate->filename : "";
}

const char* mjsim_loadError(mjsim* h) {
  return h->simulate ? h->simulate->load_error : "";
}

void mjsim_setLoadError(mjsim* h, const char* error) {
  if (!h->simulate) return;
  const mujoco::MutexLock lock(h->simulate->mtx);
  const auto max_length = sizeof(h->simulate->load_error);
  std::strncpy(h->simulate->load_error, error, max_length - 1);
  h->simulate->load_error[max_length - 1] = '\0';
}

double mjsim_ctrlNoiseStd(mjsim* h) {
  return h->simulate ? h->simulate->ctrl_noise_std : 0;
}

double mjsim_ctrlNoiseRate(mjsim* h) {
  return h->simulate ? h->simulate->ctrl_noise_rate : 0;
}

int mjsim_realTimeIndex(mjsim* h) {
  return h->simulate ? h->simulate->real_time_index : 0;
}

// slow-down factor of the selected real-time index, in percent
float mjsim_percentRealTime(int index) {
  return mujoco::Simulate::percentRealTime[index];
}

int mjsim_speedChanged(mjsim* h) {
  return h->simulate ? h->simulate->speed_changed : 0;
}

void mjsim_setSpeedChanged(mjsim* h, int speed_changed) {
  if (h->simulate) h->simulate->speed_changed = speed_changed != 0;
}

float mjsim_measuredSlowdown(mjsim* h) {
  return h->simulate ? h->simulate->measured_slowdown : 1;
}

void mjsim_setMeasuredSlowdown(mjsim* h, float measured_slowdown) {
  if (h->simulate) h->simulate->measured_slowdown = measured_slowdown;
}

int mjsim_refreshRate(mjsim* h) {
  return h->simulate ? h->simulate->refresh_rate : 60;
}

int mjsim_busywait(mjsim* h) {
  return h->simulate ? h->simulate->busywait : 0;
}

int mjsim_ui0Enable(mjsim* h) { return h->simulate ? h->simulate->ui0_enable : 0; }
int mjsim_ui1Enable(mjsim* h) { return h->simulate ? h->simulate->ui1_enable : 0; }

void mjsim_setUi0Enable(mjsim* h, int enabled) {
  if (h->simulate) h->simulate->ui0_enable = enabled;
}

void mjsim_setUi1Enable(mjsim* h, int enabled) {
  if (h->simulate) h->simulate->ui1_enable = enabled;
}

// Not in the Python binding: the UI color scheme (0 default, 1 orange, 2 white,
// 3 black), applied when the UI loop starts.
void mjsim_setColor(mjsim* h, int color) {
  if (h->simulate) h->simulate->color = color;
}

}  // extern "C"
