#include <jni.h>        // JNI header provided by JDK
#include <stdio.h>      // C Standard IO Header
#include "PicamJNI.h"   // Generated

// Implementation of the native method sayHello()
JNIEXPORT void JNICALL Java_org_photonvision_raspi_PicamJNI_createCamera(JNIEnv *env, jobject thisObj) {
   printf("Hello World!\n");
   return;
}