/*
Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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