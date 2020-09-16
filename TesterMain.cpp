
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <iostream>
#include <thread>

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

  Java_org_photonvision_raspi_PicamJNI_setShouldCopyColor(nullptr, nullptr,
                                                          false);

  auto start_time = std::chrono::steady_clock::now();
  auto last_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 500ms) {
    cv::Mat *mat = reinterpret_cast<cv::Mat *>(
        Java_org_photonvision_raspi_PicamJNI_grabFrame(nullptr, nullptr,
                                                       false));
    std::cout << "dt: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - last_time)
                     .count()
              << std::endl;
    last_time = std::chrono::steady_clock::now();

    std::cout << "latency: "
              << Java_org_photonvision_raspi_PicamJNI_getFrameLatency(nullptr,
                                                                      nullptr)
              << std::endl;

    mat->release();
  }
  std::cout << "========== writing ========" << std::endl;
  jlong mat_ptr =
      Java_org_photonvision_raspi_PicamJNI_grabFrame(nullptr, nullptr, true);
  if (mat_ptr == 0) {
    std::cerr << "Got nullptr from grab frame" << std::endl;
    return;
  }
  cv::Mat *mat = reinterpret_cast<cv::Mat *>(mat_ptr);
  bool success = cv::imwrite("out.png", *mat);
  if (!success)
    std::cerr << "whoops" << std::endl;
  std::cout << "releasing" << std::endl;
  mat->release();
}

// This is just a little wrapper for manual testing that calls the functions
// that would ususally get called from Java. It's useful because Gradle startup
// on the Pi 3 takes *forever*.
int main(int argc, char *argv[]) {
  int width = 320, height = 240, fps = 120;
  if (argc >= 4) {
    width = atoi(argv[1]);
    height = atoi(argv[2]);
    fps = atoi(argv[3]);
  }

  Java_org_photonvision_raspi_PicamJNI_createCamera(nullptr, nullptr, width,
                                                    height, fps);
  print();
  Java_org_photonvision_raspi_PicamJNI_destroyCamera(nullptr, nullptr);

  Java_org_photonvision_raspi_PicamJNI_createCamera(nullptr, nullptr, width / 2, height / 2, 60);
  print();
  // Java_org_photonvision_raspi_PicamJNI_destroyCamera(nullptr, nullptr);
}
