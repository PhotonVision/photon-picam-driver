
#include <opencv2/core/core.hpp>

#include <iostream>
#include <chrono>

#include "PicamJNI.hpp"

extern "C" {
#include "RaspiCamControl.h"
#include "RaspiTex.h"

#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_parameters_camera.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
}

void print() {
  using namespace std::chrono_literals;

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 30s) {
    cv::Mat *mat = reinterpret_cast<cv::Mat *>(
        Java_org_photonvision_raspi_PicamJNI_grabFrame(nullptr, nullptr));
    std::cout << static_cast<unsigned int>(mat->at<unsigned char>(0, 0))
              << " "
              << static_cast<unsigned int>(mat->at<unsigned char>(1, 0))
              << " "
              << static_cast<unsigned int>(mat->at<unsigned char>(2, 0))
              << std::endl;
    mat->release();
  }
}

// This is just a little wrapper for manual testing that calls the functions
// that would ususally get called from Java. It's useful because Gradle startup
// on the Pi 3 takes *forever*.
int main(int argc, char *argv[]) {
  int width = 1920, height = 1080;
  if (argc >= 3) {
    width = atoi(argv[1]);
    height = atoi(argv[2]);
  }

  Java_org_photonvision_raspi_PicamJNI_createCamera(nullptr, nullptr, width,
                                                    height, 60);
  print();
  Java_org_photonvision_raspi_PicamJNI_destroyCamera(nullptr, nullptr);

  // Java_org_photonvision_raspi_PicamJNI_createCamera(nullptr, nullptr, width / 2,
  //                                                   height / 2, 60);
  // print();
  // Java_org_photonvision_raspi_PicamJNI_destroyCamera(nullptr, nullptr);
}