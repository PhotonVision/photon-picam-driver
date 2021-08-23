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

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>

#include "PicamJNI.hpp" // Generated

extern "C" {

#include <jni.h>

#include "RaspiCamControl.h"
#include "RaspiHelpers.h"
#include "RaspiTex.h"

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

RASPITEX_STATE tex_state{};
MMAL_STATE mmal_state{};

std::mutex mat_available_mutex;
std::condition_variable mat_available;

// The below buffers hold the uncropped color and threshold channels picked out
// of the VCSM buffer (yeah yeah, raw pointers bad, but GCC doesn't vectorize
// anything else)
cv::Mat color_mat{};
cv::Mat threshold_mat{};
std::vector<unsigned char> inter_cropped_buffer;

std::thread mat_thread;

bool copy_color = false;  // Protected by mat_available_mutex
unsigned int current_fps; // Not protected by any mutex because the camera needs
                          // to be destroyed for this to change

std::mutex timestamp_mutex;
uint64_t last_stc_timestamp;

std::array<std::mutex, NUM_FRAMEBUFFERS> vcsm_mutexes;
std::array<std::vector<unsigned char>, NUM_FRAMEBUFFERS> intermediate_buffers;
unsigned char *vcsm_buffer;

std::mutex hsv_uniforms_mutex;
std::array<double, 6> hsv_thresholds = {0, 0, 0, 1, 1, 0.5};

namespace {
void setup_mmal(MMAL_STATE *state, RASPICAM_CAMERA_PARAMETERS *cam_params,
                unsigned int width, unsigned int height, unsigned int fps) {
  int status;

  bcm_host_init();

  current_fps = fps;

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
        .num_preview_video_frames = width * height >= 1920 * 1080
                                        ? 3
                                        : 3 + vcos_max(0, (fps - 30) / 10),
        .stills_capture_circular_buffer_height = 0,
        .fast_preview_resume = 0,
        .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC};
    mmal_port_parameter_set(state->camera->control, &cam_config.hdr);
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

  state->format->es->video.frame_rate.num = fps;
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

void enqueue_mat(unsigned char *vcsm_buffer, int fbo_idx, int width, int height,
                 int fb_width, int fb_height) {
  // This seems fast enough on Linux/libpthread so no threadpool for us
  std::thread t([=] {
    std::scoped_lock lk(vcsm_mutexes[fbo_idx]);

    auto inter_cropped_buffer = intermediate_buffers[fbo_idx];
    inter_cropped_buffer.clear();
    inter_cropped_buffer.reserve(width * height * 4);
    unsigned char *inter_cropped_buffer_data =
        inter_cropped_buffer
            .data(); // Must be outside of loop for autovectorization

    {
      size_t line_size_cropped = width * 4;
      size_t line_size_uncropped = fb_width * 4;

      for (int y = 0; y < height; y++) {
        std::memcpy(inter_cropped_buffer_data + y * line_size_cropped,
                    vcsm_buffer + y * line_size_uncropped, line_size_cropped);
      }

      vcsm_unlock_ptr(vcsm_buffer);
    }

    {
      std::scoped_lock lk(mat_available_mutex);

      // Once again needs to be calculated outside of the loop for the loop to
      // get vectorized
      int bound = width * height;

      threshold_mat = cv::Mat(height, width, CV_8UC1);
      unsigned char *threshold_out_buf = threshold_mat.data;

      if (copy_color) {
        color_mat = cv::Mat(height, width, CV_8UC3);
        unsigned char *color_out_buf = color_mat.data;
        for (int i = 0; i < bound; i++) {
          std::memcpy(color_out_buf + i * 3, inter_cropped_buffer_data + i * 4,
                      3);
          threshold_out_buf[i] = inter_cropped_buffer_data[i * 4 + 3];
        }
      } else {
        for (int i = 0; i < bound; i++) {
          threshold_out_buf[i] = inter_cropped_buffer_data[i * 4 + 3];
        }
      }
    }

    mat_available.notify_all();
  });
  t.detach();
}

void set_last_timestamp(uint64_t stc_timestamp) {
  std::scoped_lock lk(timestamp_mutex);
  last_stc_timestamp = stc_timestamp;
}

void wait_for_vcsm_read_done(int fbo_idx) {
  std::scoped_lock lk(vcsm_mutexes[fbo_idx]);
}

void get_thresholds(double lower[3], double upper[3]) {
  std::scoped_lock lk(hsv_uniforms_mutex);
  std::copy(hsv_thresholds.begin(), hsv_thresholds.begin() + 3, lower);
  std::copy(hsv_thresholds.begin() + 3, hsv_thresholds.end(), upper);
}
} // namespace

JNIEXPORT jstring
Java_org_photonvision_raspi_PicamJNI_getSensorModelRaw(JNIEnv *env, jclass) {
  static constexpr int camera_num = 0; // We only support one CSI camera
  char camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN]{};
  get_sensor_name(camera_num, camera_name);

  return env->NewStringUTF(camera_name);
}

JNIEXPORT jboolean
Java_org_photonvision_raspi_PicamJNI_isVCSMSupported(JNIEnv *, jclass) {
  // Memoize the return code so we're not calling init all the time
  static int rc = vcsm_init();
  return rc;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(
    JNIEnv *, jclass, jint width, jint height, jint fps) {
  try {
    if (width < 0 || height < 0 || fps < 0) {
      throw std::runtime_error{"Width, height, and FPS must be positive"};
    }

    std::cout << "Setting up MMAL, EGL, and OpenGL for " << width << "x"
              << height << std::endl;

    int ret;

    raspitex_set_defaults(&tex_state);
    tex_state.width = static_cast<unsigned int>(width);
    tex_state.height = static_cast<unsigned int>(height);
    tex_state.enqueue_mat = enqueue_mat;
    tex_state.wait_for_vcsm_read_done = wait_for_vcsm_read_done;
    tex_state.set_last_frame_timestamp = set_last_timestamp;
    tex_state.get_thresholds = get_thresholds;
    ret = raspitex_init(&tex_state);
    if (ret != 0) {
      throw std::runtime_error{
          "Couldn't initialize OpenGL and DispmanX native window"};
    }

    RASPICAM_CAMERA_PARAMETERS cam_params{};
    raspicamcontrol_set_defaults(&cam_params);

    setup_mmal(&mmal_state, &cam_params, tex_state.width, tex_state.height,
               fps); // Throws

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

    raspicamcontrol_set_flips(mmal_state.camera, true, false);

    return false;
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return true;
  }
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_destroyCamera(JNIEnv *, jclass) {
  raspitex_stop(&tex_state);

  {
    // Yuuuge hack... mmal_vc_port_send_callback gets called when buffers come
    // back from the VC, and it doesn't check to see if we've freed the port
    // that the callback is associated with. If the VC starts processing a
    // buffer before we stop everything, and then returns the buffer after we've
    // stopped then there's the possibility that it'll get returned after all
    // the resources that are used to handle it have been freed, which is UB.
    // Ideally we'd have a nullptr check there, but alas, we can't easily patch
    // that code. Waiting to make sure that all buffers get processed before we
    // free is a solution, albeit a shitty one.
    // After we release we can look into patching the issue and PRing it
    // upstream.

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
  }

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
    JNIEnv *, jclass, jdouble h_l, jdouble s_l, jdouble v_l, jdouble h_u,
    jdouble s_u, jdouble v_u) {
  std::scoped_lock lk(hsv_uniforms_mutex);
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
  constexpr int padding_microseconds = 1000;

  if (!mmal_state.camera)
    return true;
  return raspicamcontrol_set_shutter_speed(
      mmal_state.camera, padding_microseconds + ((double)exposure / 100.0) *
                                                    (1e6 / current_fps -
                                                     2 * padding_microseconds));
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setBrightness(
    JNIEnv *, jclass, jint brightness) {
  if (!mmal_state.camera)
    return true;
  return raspicamcontrol_set_brightness(mmal_state.camera, brightness);
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_setGain(JNIEnv *, jclass, jint gain) {
  if (!mmal_state.camera)
    return true;
  // Right now we only expose one parameter
  // Value ranges from here:
  // https://picamera.readthedocs.io/en/release-1.10/api_camera.html#picamera.camera.PiCamera.awb_gains
  return raspicamcontrol_set_awb_mode(mmal_state.camera, MMAL_PARAM_AWBMODE_OFF) 
    || raspicamcontrol_set_awb_gains(mmal_state.camera, gain / 100.0 * 8.0,
                                   gain / 100.0 * 8.0);
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setRotation(
    JNIEnv *, jclass, jint rotationOrdinal) {
  int rotation = (rotationOrdinal + 3) * 90; // Degrees
  if (!mmal_state.camera)
    return true;
  else if (tex_state.preview_rotation == rotation)
    return false;
  tex_state.preview_rotation = rotation;
  return raspicamcontrol_set_rotation(mmal_state.camera, rotation);
}

JNIEXPORT void JNICALL Java_org_photonvision_raspi_PicamJNI_setShouldCopyColor(
    JNIEnv *, jclass, jboolean should_copy_color) {
  std::scoped_lock lk(mat_available_mutex);
  copy_color = should_copy_color;
}

JNIEXPORT jlong JNICALL
Java_org_photonvision_raspi_PicamJNI_getFrameLatency(JNIEnv *, jclass) {
  if (!mmal_state.camera_preview_port)
    return 0;

  uint64_t current_stc_timestamp;
  mmal_port_parameter_get_uint64(mmal_state.camera_preview_port,
                                 MMAL_PARAMETER_SYSTEM_TIME,
                                 &current_stc_timestamp);

  std::scoped_lock lk(timestamp_mutex);
  return std::max(
      static_cast<int64_t>(current_stc_timestamp - last_stc_timestamp),
      INT64_C(0));
}

JNIEXPORT jlong JNICALL Java_org_photonvision_raspi_PicamJNI_grabFrame(
    JNIEnv *, jclass, jboolean should_return_color) {
  {
    jlong ret = 0;

    std::unique_lock lk(mat_available_mutex);
    if (!should_return_color)
      // We don't care about waiting for a new frame when returning the color
      // Mat because we assume that we've already just waited for a new frame
      // when the Java code grabbed the threshold Mat. We also assume that this
      // with should_return_color = true will only be called once after each
      // call with should_return_color = false, because otherwise the Mat we
      // return will have already been released by the Java code. Caveat emptor.
      mat_available.wait(lk);

    // Mat is released by Java code
    if (!color_mat.empty() && should_return_color)
      ret = reinterpret_cast<jlong>(new cv::Mat(color_mat));
    else if (!should_return_color)
      ret = reinterpret_cast<jlong>(new cv::Mat(threshold_mat));

    return ret;
  }
}

} // extern "C"
