/*
 * Copyright (C) 2020 Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <opencv2/core/core.hpp>

#include <cmath>
#include <condition_variable>
#include <iostream>
#include <future>
#include <mutex>
#include <shared_mutex>

#include "PicamJNI.hpp" // Generated

extern "C" {

#include <jni.h>

#include "RaspiCamControl.h"
#include "RaspiTex.h"
#include "RaspiHelpers.h"

#include <bcm_host.h>

#include <interface/vcsm/user-vcsm.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_parameters_camera.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util_params.h>

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(void *) <= sizeof(jlong));

struct MMAL_STATE {
  MMAL_COMPONENT_T *camera;
  MMAL_COMPONENT_T *preview;
  MMAL_ES_FORMAT_T *format;
  MMAL_STATUS_T status;
  MMAL_PORT_T *camera_preview_port, *camera_video_port, *camera_still_port;
  MMAL_PORT_T *preview_input_port;

  MMAL_CONNECTION_T *camera_preview_connection;
};

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_STILLS_PORT 2

constexpr int video_framerate = 120; // This is really a max framerate

RASPITEX_STATE tex_state{};
MMAL_STATE mmal_state{};

std::mutex mat_available_mutex;
std::condition_variable mat_available;
std::thread mat_thread;
std::atomic_bool new_frame_available;
// The below buffers hold the uncropped color and threshold channels picked out of the VCSM buffer (yeah yeah, raw pointers bad, but GCC doesn't vectorize anything else)
cv::Mat color_mat;
cv::Mat threshold_mat;
unsigned char *inter_color_buffer;
unsigned char *inter_threshold_buffer;

std::array<std::shared_mutex, NUM_FRAMEBUFFERS> vcsm_mutexes;
unsigned char *vcsm_buffer;

std::mutex hsv_uniforms_mutex;
std::array<double, 6> hsv_thresholds = {0, 0, 0, 1, 1, 0.5};

namespace {
void setup_mmal(MMAL_STATE *state, RASPICAM_CAMERA_PARAMETERS *cam_params,
                unsigned int width, unsigned int height) {
  int status;

  bcm_host_init();

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &state->camera);
  if (status != MMAL_SUCCESS) {
    std::ostringstream msg;
    msg << "Couldn't create MMAL camera component : error " << status;
    throw std::runtime_error{msg.str()};
  }

  MMAL_PARAMETER_INT32_T camera_num = {
      {MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)},
      0 /* camera num of zero */};

  status = mmal_port_parameter_set(state->camera->control, &camera_num.hdr);
  if (status != MMAL_SUCCESS) {
    std::ostringstream msg;
    msg << "Couldn't set MMAL camera component number : error " << status;
    throw std::runtime_error{msg.str()};
  }
  if (!state->camera->output_num) {
    status = MMAL_ENOSYS;
    throw std::runtime_error{"Camera doesn't have any output ports"};
  }

  status = mmal_port_parameter_set_uint32(
      state->camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG,
      0 /* automatic sensor mode selection */);
  if (status != MMAL_SUCCESS) {
    std::ostringstream msg;
    msg << "Couldn't set camera sensor mode : error " << status;
    throw std::runtime_error{msg.str()};
  }

  state->camera_preview_port = state->camera->output[MMAL_CAMERA_PREVIEW_PORT];
  state->camera_video_port = state->camera->output[MMAL_CAMERA_VIDEO_PORT];
  state->camera_still_port = state->camera->output[MMAL_CAMERA_STILLS_PORT];

  {
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
        {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
        .max_stills_w = width,
        .max_stills_h = height,
        .stills_yuv422 = 0,
        .one_shot_stills = 1,
        .max_preview_video_w = width,
        .max_preview_video_h = height,
        .num_preview_video_frames = 3 + vcos_max(0, (video_framerate-30)/10),
        .stills_capture_circular_buffer_height = 0,
        .fast_preview_resume = 0,
        .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC};
    mmal_port_parameter_set(state->camera->control, &cam_config.hdr);
  }

  {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         {15, 1}, {120, 1}
      };
      mmal_port_parameter_set(state->camera_preview_port, &fps_range.hdr);
  }

  status = raspicamcontrol_set_all_parameters(state->camera, cam_params);
  if (status != 0)
    throw std::runtime_error{"Couldn't set all camera parameters"};

  state->format = state->camera_preview_port->format;

  state->format->encoding = MMAL_ENCODING_OPAQUE;
  state->format->encoding_variant = MMAL_ENCODING_I420;

  state->format->es->video.width = VCOS_ALIGN_UP(width, 32);
  state->format->es->video.height = VCOS_ALIGN_UP(height, 16);
  state->format->es->video.crop.x = 0;
  state->format->es->video.crop.y = 0;
  state->format->es->video.crop.width = VCOS_ALIGN_UP(width, 32);
  state->format->es->video.crop.height = VCOS_ALIGN_UP(height, 16);

  state->format->es->video.frame_rate.num = video_framerate;
  state->format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(state->camera_preview_port);
  if (status != MMAL_SUCCESS) {
    throw std::runtime_error{"Preview port format couldn't be set"};
  }

  status = mmal_component_enable(state->camera);
  if (status != MMAL_SUCCESS) {
    throw std::runtime_error{"Couldn't enable camera component"};
  }
}

void enqueue_mat(unsigned char *vcsm_buf, int fbo_idx, int width, int height, int fb_width, int fb_height) {
  std::shared_lock<std::shared_mutex> lk(vcsm_mutexes[fbo_idx]);

  if (!inter_color_buffer) {
    // Color buffer is RGB, so 3 bytes per pixel
    inter_color_buffer = new unsigned char[fb_width * fb_height * 3];
    inter_threshold_buffer = new unsigned char[fb_width * fb_height];
  }

  std::thread t([=] {
    {
      std::shared_lock<std::shared_mutex> lk(vcsm_mutexes[fbo_idx]);
      int bound = fb_width * (height - 1) + width;
      for (int i = 0; i < bound; i++) {
        // inter_color_buffer[i * 3] = vcsm_buf[i * 4];
        // inter_color_buffer[i * 3 + 1] = vcsm_buf[i * 4 + 1];
        // inter_color_buffer[i * 3 + 2] = vcsm_buf[i * 4 + 2];
        inter_threshold_buffer[i] = vcsm_buf[i * 4 + 3];
      }
      vcsm_unlock_ptr(vcsm_buf);
    }

    {
      std::scoped_lock<std::mutex> lk(mat_available_mutex);

      // color_mat = cv::Mat(height, width, CV_8UC3);
      threshold_mat = cv::Mat(height, width, CV_8UC1);
      // unsigned char *color_out_buf = color_mat.data;
      unsigned char *threshold_out_buf = threshold_mat.data;
      for (int y = 0; y < height; y++) {
        // std::memcpy(color_out_buf + y * width, inter_color_buffer + y * fb_width * 3,
        //           width * 3);
        std::memcpy(threshold_out_buf + y * width, inter_threshold_buffer + y * fb_width,
                  width);
      }
    }

    mat_available.notify_all();
  });
  t.detach();
}

void wait_for_vcsm_read_done(int fbo_idx) {
  // Acquire exclusive lock, which makes us block until the shared lock is unlocked
  std::scoped_lock<std::shared_mutex> lk(vcsm_mutexes[fbo_idx]);
}

void get_thresholds(double lower[3], double upper[3]) {
  std::scoped_lock<std::mutex> lk(hsv_uniforms_mutex);
  std::copy(hsv_thresholds.begin(), hsv_thresholds.begin() + 3, lower);
  std::copy(hsv_thresholds.begin() + 3, hsv_thresholds.end(), upper);
}
} // namespace

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(
    JNIEnv *, jclass, jint width, jint height, jint fps) {
  try {
    if (width < 0 || height < 0 || fps < 0) {
      throw std::runtime_error{"Width, height, and FPS must be positive"};
    }

    std::cout << "Setting up MMAL, EGL, and OpenGL for " << width << "x" << height << std::endl;

    int ret;

    raspitex_set_defaults(&tex_state);
    tex_state.width = static_cast<unsigned int>(width);
    tex_state.height = static_cast<unsigned int>(height);
    tex_state.enqueue_mat = enqueue_mat;
    tex_state.wait_for_vcsm_read_done = wait_for_vcsm_read_done;
    tex_state.get_thresholds = get_thresholds;
    ret = raspitex_init(&tex_state);
    if (ret != 0) {
      throw std::runtime_error{
          "Couldn't initialize OpenGL and DispmanX native window"};
    }

    RASPICAM_CAMERA_PARAMETERS cam_params{};
    raspicamcontrol_set_defaults(&cam_params);

    setup_mmal(&mmal_state, &cam_params, tex_state.width,
               tex_state.height); // Throws

    ret = raspitex_configure_preview_port(&tex_state,
                                          mmal_state.camera_preview_port);
    if (ret != 0) {
      throw std::runtime_error{"Couldn't configure MMAL preview port"};
    }

    std::cout << "Setup done; starting OpenGL and capture worker" << std::endl;

    ret = raspitex_start(&tex_state);
    if (ret != 0) {
      throw std::runtime_error{"Couldn't start capture worker"};
    }

    return false;
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return true;
  }
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_destroyCamera(JNIEnv *, jclass) {
  raspitex_stop(&tex_state);
  raspitex_destroy(&tex_state);

  // Disable all ports not handled by connections
  check_disable_port(mmal_state.camera_video_port);
  check_disable_port(mmal_state.camera_still_port);

  // Disable connections
  if (mmal_state.camera_preview_connection)
    mmal_connection_destroy(mmal_state.camera_preview_connection);

  // Disable components
  if (mmal_state.camera)
    mmal_component_disable(mmal_state.camera);
  if (mmal_state.preview)
    mmal_component_disable(mmal_state.preview);

  // Destroy the camera component
  if (mmal_state.camera) {
    mmal_component_destroy(mmal_state.camera);
    mmal_state.camera = nullptr;
  }

  return false;
}

JNIEXPORT void JNICALL Java_org_photonvision_raspi_PicamJNI_setThresholds(
    JNIEnv *, jclass, jdouble h_l, jdouble s_l, jdouble v_l, jdouble h_u, jdouble s_u, jdouble v_u) {
  std::scoped_lock<std::mutex> lk(hsv_uniforms_mutex);
  // You _can_ pass a jdouble[], but it's slow and unpleasant
  hsv_thresholds[0] = h_l;
  hsv_thresholds[1] = s_l;
  hsv_thresholds[2] = v_l;
  hsv_thresholds[3] = h_u;
  hsv_thresholds[4] = s_u;
  hsv_thresholds[5] = v_u;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setExposure(
    JNIEnv *, jclass, jint exposure) {
  return raspicamcontrol_set_exposure_compensation(mmal_state.camera, exposure);
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setBrightness(
    JNIEnv *, jclass, jint brightness) {
  return raspicamcontrol_set_brightness(mmal_state.camera, brightness);
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_setGain(JNIEnv *, jclass, jint gain) {
  // TODO: this takes two parameters, but V4L2/cscore only exposes one
  return raspicamcontrol_set_gains(mmal_state.camera, gain, gain);
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setRotation(
    JNIEnv *, jclass, jint rotation) {
  return false;
}

JNIEXPORT jlong JNICALL Java_org_photonvision_raspi_PicamJNI_grabFrame(JNIEnv *,
                                                                       jclass) {
  {
    std::unique_lock<std::mutex> lk(mat_available_mutex);
    mat_available.wait(lk);

    return reinterpret_cast<jlong>(new cv::Mat(threshold_mat));
  }
}

} // extern "C"
