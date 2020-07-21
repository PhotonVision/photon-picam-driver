#include <opencv2/core/core.hpp>

extern "C" {

#include <jni.h>        // JNI header provided by JDK
#include <stdio.h>      // C Standard IO Header
#include "PicamJNI.h"   // Generated

#include "triangle.h"

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(JNIEnv *, jclass) {
  printf("Hello World from native code!!\n");

  triangleMain();

  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_destroyCamera
  (JNIEnv *, jclass) {
   printf("Freeing picam resource...\n");
   return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setExposure
  (JNIEnv *, jclass, jint exposure) {
  printf("Setting exposure to %d\n", exposure);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setBrightness
  (JNIEnv *, jclass, jint brightness) {
  printf("Setting brightness to %d\n", brightness);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setIso
  (JNIEnv *, jclass, jint iso) {
  printf("Setting iso to %d\n", iso);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setRotation
  (JNIEnv *, jclass, jint rotation) {
  printf("Setting rotation to %d\n", rotation);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setVideoMode
  (JNIEnv *, jclass, jint width, jint height, jint fps) {
  printf("Setting video mode to videomode: %d x %d at %d fps\n", width, height, fps);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_grabFrame
  (JNIEnv *, jclass, jlong imageNativeObj) {
  printf("Grabbing from Mat @ %llu\n", imageNativeObj);

  cv::Mat& image = *((cv::Mat*)imageNativeObj);
  
  cv::Size s = image.size();
  printf("Got mat of size %d x %d\n", s.width, s.height);
  return true;
}

}  // extern "C"