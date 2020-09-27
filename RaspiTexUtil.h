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
 
#ifndef RASPITEX_UTIL_H_
#define RASPITEX_UTIL_H_

#define VCOS_LOG_CATEGORY (&raspitex_log_category)
#include "RaspiTex.h"
#include "interface/vcos/vcos.h"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern VCOS_LOG_CAT_T raspitex_log_category;

#define SHADER_MAX_ATTRIBUTES 16
#define SHADER_MAX_UNIFORMS 16
/**
 * Container for a simple shader program. The uniform and attribute locations
 * are automatically setup by raspitex_build_shader_program.
 */
typedef struct RASPITEXUTIL_SHADER_PROGRAM_T {
  const char *vertex_source;   /// Pointer to vertex shader source
  const char *fragment_source; /// Pointer to fragment shader source

  /// Array of uniform names for raspitex_build_shader_program to process
  const char *uniform_names[SHADER_MAX_UNIFORMS];
  /// Array of attribute names for raspitex_build_shader_program to process
  const char *attribute_names[SHADER_MAX_ATTRIBUTES];

  GLint vs;      /// Vertex shader handle
  GLint fs;      /// Fragment shader handle
  GLint program; /// Shader program handle

  /// The locations for uniforms defined in uniform_names
  GLint uniform_locations[SHADER_MAX_UNIFORMS];

  /// The locations for attributes defined in attribute_names
  GLint attribute_locations[SHADER_MAX_ATTRIBUTES];
} RASPITEXUTIL_SHADER_PROGRAM_T;

/* Uncomment to enable extra GL error checking */
//#define CHECK_GL_ERRORS
#if defined(CHECK_GL_ERRORS)
#define GLCHK(X)                                                               \
  do {                                                                         \
    GLenum err = GL_NO_ERROR;                                                  \
    X;                                                                         \
    while ((err = glGetError())) {                                             \
      vcos_log_error("GL error 0x%x in " #X "file %s line %d", err, __FILE__,  \
                     __LINE__);                                                \
      vcos_assert(err == GL_NO_ERROR);                                         \
      exit(err);                                                               \
    }                                                                          \
  } while (0)
#else
#define GLCHK(X) X
#endif /* CHECK_GL_ERRORS */

/* Benchmarking */
double get_wall_time(void);

/* Default GL scene ops functions */
int raspitexutil_create_native_window(RASPITEX_STATE *raspitex_state);
int raspitexutil_gl_init_1_0(RASPITEX_STATE *raspitex_state);
int raspitexutil_gl_init_2_0(RASPITEX_STATE *raspitex_state);
int raspitexutil_update_model(RASPITEX_STATE *raspitex_state);
int raspitexutil_redraw(RASPITEX_STATE *raspitex_state);
void raspitexutil_gl_term(RASPITEX_STATE *raspitex_state);
void raspitexutil_destroy_native_window(RASPITEX_STATE *raspitex_state);
int raspitexutil_create_textures(RASPITEX_STATE *raspitex_state);
int raspitexutil_update_texture(RASPITEX_STATE *raspitex_state,
                                EGLClientBuffer mm_buf);
int raspitexutil_update_y_texture(RASPITEX_STATE *raspitex_state,
                                  EGLClientBuffer mm_buf);
int raspitexutil_update_u_texture(RASPITEX_STATE *raspitex_state,
                                  EGLClientBuffer mm_buf);
int raspitexutil_update_v_texture(RASPITEX_STATE *raspitex_state,
                                  EGLClientBuffer mm_buf);
int raspitexutil_capture_bgra(struct RASPITEX_STATE *state, uint8_t **buffer,
                              size_t *buffer_size);
void raspitexutil_close(RASPITEX_STATE *raspitex_state);

/* Utility functions */
int raspitexutil_build_shader_program(RASPITEXUTIL_SHADER_PROGRAM_T *p);
void raspitexutil_brga_to_rgba(uint8_t *buffer, size_t size);

#endif /* RASPITEX_UTIL_H_ */
