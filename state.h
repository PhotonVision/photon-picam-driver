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

#ifndef __STATE_H
#define __STATE_H

#include <semaphore.h>

#include <GLES/gl.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <EGL/eglext_brcm.h>

typedef struct {
   // Screen size (no alignment gurantees)
   int screenWidth;
   int screenHeight;

   // VCSM stuff (buffer must be power of two aligned and is cropped after being copied)
   unsigned int vcsmBufWidth;
   unsigned int vcsmBufHeight;
   struct egl_image_brcm_vcsm_info vcsmInfo;
   unsigned char intermediateBuffer[2048 * 2048 * 4]; // Used when copying out and cropping shared memory

   // Used by OMX
   void *eglImage;
   sem_t fillBufferDone;
} ProgramState;

#endif
