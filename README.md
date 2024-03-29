# photon-picam-driver
Driver for the raspberry Pi camera. This driver provides access to the Pi camera through MMAL and allows OpenGL to directly access the camera. This is only supported on Raspberry Pi 3-based systems (e.g Pi 3 Model B, CM 3.)

## Installation
 1. Clone this repo in `/opt/vc/src/hello_pi/`.
 2. Run `/opt/vc/src/hello_pi/rebuild.sh`.
 3. Run `sudo apt install libopencv-core-dev`.
 4. Run `sudo raspi-config` and install and enable the legacy OpenGL driver.
 4. Run `make` in the top level directory (`/opt/vc/src/hello_pi/photon-picam-driver`) of this project (you may want to pass `-j` flags.)

## Contribution
Please use `clang-format` and with the "llvm" style before you PR your code. You can run `clang-format -i -style=llvm *.cpp *.hpp *.c *.h` to format all files in the project.
