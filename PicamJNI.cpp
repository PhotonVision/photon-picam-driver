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
#include <iostream>

#include "PicamJNI.hpp" // Generated

extern "C" {

#include <jni.h>

#include <bcm_host.h>
#include <interface/vcsm/user-vcsm.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <pthread.h>

#include "state.h"
#include "video.h"

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(void *) <= sizeof(jlong));

static ProgramState state{};
static pthread_t omxThread;

unsigned int nextPowerOfTwo(unsigned int num) {
  return static_cast<unsigned int>(std::pow(2, std::ceil(std::log2(num))));
}

JNIEXPORT jlong JNICALL Java_org_photonvision_raspi_PicamJNI_initVCSMInfo(
    JNIEnv *, jclass, jint width, jint height) {
  vcsm_init();

  state.vcsmBufWidth = nextPowerOfTwo(width);
  state.vcsmBufHeight = nextPowerOfTwo(height);
  state.vcsmInfo.width = state.vcsmBufWidth;
  state.vcsmInfo.height = state.vcsmBufHeight;

  if (state.vcsmBufWidth > 2048 || state.vcsmBufHeight > 2048) {
    std::cerr << "VCSM doesn't support buffers with a width or height greater "
                 "than 2048\n";
    return 0;
  }

  return reinterpret_cast<jlong>(&state.vcsmInfo);
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_setEGLImageHandle(JNIEnv *, jclass,
                                                       jlong eglImage) {
  if (eglImage < 0) {
    std::cerr << "EGLImage ID cannot be negative\n";
    return true;
  }

  state.eglImage = reinterpret_cast<void *>(eglImage);

  return false;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(
    JNIEnv *, jclass, jint width, jint height, jint fps) {
  try {
    if (width < 0 || height < 0 || fps < 0) {
      throw std::runtime_error{"Width, height, and FPS must be positive"};
    }
    state.screenWidth = width;
    state.screenHeight = height;

    sem_init(&state.fillBufferDone, 0, 1);

    // Init Broadcom libs (this has probably already been done by JOGL, but
    // we'll be defensive)
    bcm_host_init();

    std::cout << "bcm_host_init done; creating OMX thread\n";

    pthread_create(&omxThread, nullptr, video_decode_test, &state);

    std::cout << std::flush;

    return false;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return true;
  }
}

JNIEXPORT void JNICALL
Java_org_photonvision_raspi_PicamJNI_waitForOMXFillBufferDone(JNIEnv *,
                                                              jclass) {
  sem_wait(&state.fillBufferDone);

  return;
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_destroyCamera(JNIEnv *, jclass) {
  return false;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setExposure(
    JNIEnv *, jclass, jint exposure) {
  return false;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setBrightness(
    JNIEnv *, jclass, jint brightness) {
  return false;
}

JNIEXPORT jboolean JNICALL
Java_org_photonvision_raspi_PicamJNI_setIso(JNIEnv *, jclass, jint iso) {
  return false;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setRotation(
    JNIEnv *, jclass, jint rotation) {
  return false;
}

JNIEXPORT void JNICALL Java_org_photonvision_raspi_PicamJNI_grabFrame(
    JNIEnv *, jclass, jlong imageNativeObj) {
  cv::Mat &outImage = *((cv::Mat *)imageNativeObj);

  cv::Size s = outImage.size();
  if (s.width != state.screenWidth || s.height != state.screenHeight ||
      outImage.channels() != 1) {
    std::cerr
        << "Passed Mat is incorrectly sized or has more than one channel\n";
    return;
  }

  // Make the buffer CPU addressable with host cache enabled
  VCSM_CACHE_TYPE_T cacheType;
  unsigned char *vcsmBuffer = static_cast<unsigned char *>(vcsm_lock_cache(
      state.vcsmInfo.vcsm_handle, VCSM_CACHE_TYPE_HOST, &cacheType));
  if (!vcsmBuffer) {
    std::cerr << "Failed to lock VCSM buffer\n";
    return;
  }

  // First we copy out every fourth byte (we need a single channel image, but
  // the shared memory we map can only RGBA--i.e. 4 bytes per pixel)
  int bound = state.vcsmBufWidth * (state.screenHeight - 1) +
              state.screenWidth; // IMPORTANT! Bound must be calculated here or
                                 // GCC doesn't vectorize the loop
  for (int i = 0; i < bound; i++) {
    // We keep an intermediate buffer because memcpy can't deal with overlapping
    // copies (memmove does, but it slower)
    state.intermediateBuffer[i] = vcsmBuffer[i * 4];
  }

  // Release the locked texture memory
  vcsm_unlock_ptr(vcsmBuffer);

  // Crop out the bits we don't need (this is needed because the shared memory
  // must be a power of two in both width and height)
  unsigned char *outBuf = outImage.ptr();
  for (int y = 0; y < state.screenHeight; y++) {
    std::memcpy(outBuf + y * state.screenWidth,
                state.intermediateBuffer + y * state.vcsmBufWidth,
                state.screenWidth);
  }
}

} // extern "C"