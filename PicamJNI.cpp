extern "C" {

#include <jni.h>        // JNI header provided by JDK
#include <stdio.h>      // C Standard IO Header
#include "org_photonvision_raspi_PicamJNI.h"   // Generated

#include "triangle.h"

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(JNIEnv *env, jobject thisObj) {
  printf("Hello World from native code!!\n");

  triangleMain();

  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_destroyCamera
  (JNIEnv *env, jobject thisObj) {
   printf("Freeing picam resource...\n");
   return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setExposure
  (JNIEnv *env, jobject thisObj, jint exposure) {
  printf("Setting exposure to %d\n", exposure);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setBrightness
  (JNIEnv *env, jobject thisObj, jint brightness) {
  printf("Setting brightness to %d\n", brightness);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setIso
  (JNIEnv *env, jobject thisObj, jint iso) {
  printf("Setting iso to %d\n", iso);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setRotation
  (JNIEnv *env, jobject thisObj, jint rotation) {
  printf("Setting rotation to %d\n", rotation);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_setVideoMode
  (JNIEnv *env, jobject thisObj, jint width, jint height, jint fps) {
  printf("Setting video mode to videomode: %d x %d at %d fps\n", width, height, fps);
  return true;
}

JNIEXPORT jboolean JNICALL Java_org_photonvision_raspi_PicamJNI_grabFrame
  (JNIEnv *env, jobject thisObj, jlong frameNativeObj) {
  printf("Grabbing from Mat@%ld\n", frameNativeObj);
  return true;
}

}  // extern "C"