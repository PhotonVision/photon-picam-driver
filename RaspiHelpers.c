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

#include <ctype.h>
#include <memory.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/vcos/vcos.h"

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
void check_disable_port(MMAL_PORT_T *port) {
  if (port && port->is_enabled)
    mmal_port_disable(port);
}

/**
 * Convert a MMAL status return value to a simple boolean of success
 * ALso displays a fault if code is not success
 *
 * @param status The error code to convert
 * @return 0 if status is success, 1 otherwise
 */
int mmal_status_to_int(MMAL_STATUS_T status) {
  if (status == MMAL_SUCCESS)
    return 0;
  else {
    switch (status) {
    case MMAL_ENOMEM:
      vcos_log_error("Out of memory");
      break;
    case MMAL_ENOSPC:
      vcos_log_error("Out of resources (other than memory)");
      break;
    case MMAL_EINVAL:
      vcos_log_error("Argument is invalid");
      break;
    case MMAL_ENOSYS:
      vcos_log_error("Function not implemented");
      break;
    case MMAL_ENOENT:
      vcos_log_error("No such file or directory");
      break;
    case MMAL_ENXIO:
      vcos_log_error("No such device or address");
      break;
    case MMAL_EIO:
      vcos_log_error("I/O error");
      break;
    case MMAL_ESPIPE:
      vcos_log_error("Illegal seek");
      break;
    case MMAL_ECORRUPT:
      vcos_log_error("Data is corrupt \attention FIXME: not POSIX");
      break;
    case MMAL_ENOTREADY:
      vcos_log_error("Component is not ready \attention FIXME: not POSIX");
      break;
    case MMAL_ECONFIG:
      vcos_log_error("Component is not configured \attention FIXME: not POSIX");
      break;
    case MMAL_EISCONN:
      vcos_log_error("Port is already connected ");
      break;
    case MMAL_ENOTCONN:
      vcos_log_error("Port is disconnected");
      break;
    case MMAL_EAGAIN:
      vcos_log_error("Resource temporarily unavailable. Try again later");
      break;
    case MMAL_EFAULT:
      vcos_log_error("Bad address");
      break;
    default:
      vcos_log_error("Unknown status error");
      break;
    }

    return 1;
  }
}

int get_sensor_name(int camera_num, char *camera_name) {
  MMAL_COMPONENT_T *camera_info;
  MMAL_STATUS_T status;

  // Default to the OV5647 setup
  strncpy(camera_name, "ov5647", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);

  // Try to get the camera name and maximum supported resolution
  status =
      mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
  if (status == MMAL_SUCCESS) {
    MMAL_PARAMETER_CAMERA_INFO_T param;
    param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
    param.hdr.size =
        sizeof(param) - 4; // Deliberately undersize to check firmware version
    status = mmal_port_parameter_get(camera_info->control, &param.hdr);

    if (status != MMAL_SUCCESS) {
      // Running on newer firmware
      param.hdr.size = sizeof(param);
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);
      if (status == MMAL_SUCCESS && param.num_cameras > camera_num) {
        strncpy(camera_name, param.cameras[camera_num].camera_name,
                MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
        camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN - 1] = 0;
      } else {
        vcos_log_error(
            "Cannot read camera info, keeping the defaults for OV5647");
        return 1;
      }
    } else {
      // Older firmware
      // Nothing to do here, keep the defaults for OV5647
    }

    mmal_component_destroy(camera_info);
  } else {
    vcos_log_error("Failed to create camera_info component");
    return 1;
  }

  return 0;
}
