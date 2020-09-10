/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2016, Tim Gover
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

/* Make the render output CPU accessible by defining a framebuffer texture
 * stored in a VCSM (VideoCore shared memory) EGL image.
 *
 * This example just demonstrates how to use use the APIs by using the CPU.
 * to blit an animated rectangle into frame-buffer texture in shared memory.
 *
 * A more realistic example would be to do a blur, edge-detect in GLSL then pass
 * the buffer to OpenCV. There may be some benefit in using multiple GL contexts
 * to reduce the impact of serializing operations with a glFlush.
 *
 * N.B VCSM textures are raster scan order textures. This makes it very
 * convenient to read and modify VCSM frame-buffer textures from the CPU.
 * However, if the output of the CPU stage is drawn again as a texture that
 * is rotated or scaled then it can sometimes be better to use glTexImage2D
 * to allow the driver to convert this back into the native texture format.
 *
 * Example usage
 * raspistill -p 0,0,1024,1024 -gw 0,0,1024,1024 -t 10000 --gl -gs vcsm_square
 */
/* Uncomment the next line to compare with the glReadPixels implementation. VCSM
 * should run at about 40fps with a 1024x1024 texture compared to about 20fps
 * using glReadPixels.
 */
//#define USE_READPIXELS

#include "vcsm_square.h"
#include "RaspiTex.h"
#include "RaspiTexUtil.h"
#include <interface/vcsm/user-vcsm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>

/* Draw a scaled quad showing the entire texture with the
 * origin defined as an attribute */
static RASPITEXUTIL_SHADER_PROGRAM_T vcsm_square_oes_shader = {
    .vertex_source = "#version 100\n"
                     ""
                     "attribute vec2 vertex;"
                     "varying vec2 texcoord;"
                     ""
                     "void main(void) {"
                     "   texcoord = 0.5 * (vertex + 1.0);"
                     "   gl_Position = vec4(vertex, 0.0, 1.0);"
                     "}",

    .fragment_source = "#version 100\n"
                       "#extension GL_OES_EGL_image_external : require\n"
                       ""
                       "precision lowp float;"
                       "precision lowp int;"
                       ""
                       "varying vec2 texcoord;"
                       ""
                       "uniform vec3 lowerThresh;"
                       "uniform vec3 upperThresh;"
                       "uniform samplerExternalOES tex;"
                       ""
                       "vec3 rgb2hsv(const vec3 p) {"
                       "  const vec4 H = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);"
                       // Using ternary seems to be faster than using mix and step
                       "  vec4 o = p.g < p.b ? vec4(p.bg, H.wz) : vec4(p.gb, H.xy);"
                       "  vec4 t = p.r < o.x ? vec4(o.xyw, p.r) : vec4(p.r, o.yzx);"
                       ""
                       "  float O = t.x - min(t.w, t.y);"
                       "  const float n = 1.0e-10;"
                       "  return vec3(abs(t.z + (t.w - t.y) / (6.0 * O + n)), O / (t.x + n), t.x);"
                       "}"
                       ""
                       "bool inRange(vec3 hsv) {"
                       "  bvec3 botBool = greaterThanEqual(hsv, lowerThresh);"
                       "  bvec3 topBool = lessThanEqual(hsv, upperThresh);"
                       "  return all(botBool) && all(topBool);"
                       "}"
                       ""
                       "void main(void) {"
                       "  vec3 col = texture2D(tex, texcoord).rgb;"
                       // TODO: put color into the RGB elements of the vec4
                       //  "  gl_FragColor = vec4(col.r, col.g, int(inRange(rgb2hsv(col))), (col.r + col.g + col.b) / 3.0);"
                       "  gl_FragColor = vec4(col, int(inRange(rgb2hsv(col))));"
                       "}",
    .uniform_names = {"tex", "lowerThresh", "upperThresh"},
    .attribute_names = {"vertex"},
};

static GLfloat quad_varray[] = {
    -1.0f, -1.0f, 1.0f, 1.0f, 1.0f,  -1.0f,
    -1.0f, 1.0f,  1.0f, 1.0f, -1.0f, -1.0f,
};

static GLuint quad_vbo;

typedef struct {
  struct egl_image_brcm_vcsm_info vcsm_info;
  EGLImageKHR egl_fb_image;

  GLuint tex_name;
  GLuint name;
} FRAMEBUFFER;

int current_fb_idx = 0;
FRAMEBUFFER framebuffers[NUM_FRAMEBUFFERS] = {};

// VCSM buffer dimensions must be a power of two. Use glViewPort to draw NPOT
// rectangles within the VCSM buffer.
static int fb_width;
static int fb_height;

unsigned int next_power_of_two(unsigned int num) {
  return (unsigned int)pow(2, ceil(log2(num)));
}

static const EGLint vcsm_square_egl_config_attribs[] = {EGL_RED_SIZE,
                                                        8,
                                                        EGL_GREEN_SIZE,
                                                        8,
                                                        EGL_BLUE_SIZE,
                                                        8,
                                                        EGL_ALPHA_SIZE,
                                                        8,
                                                        EGL_RENDERABLE_TYPE,
                                                        EGL_OPENGL_ES2_BIT,
                                                        EGL_NONE};

// Initializes and binds a framebuffer bound to a ELG_IMAGE_BRCM_VCSM EGLImage
static int init_framebuffer(FRAMEBUFFER *fb, RASPITEX_STATE *raspitex_state) {
  GLCHK(glGenFramebuffers(1, &fb->name));
  GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, fb->name));

  GLCHK(glGenTextures(1, &fb->tex_name));
  GLCHK(glBindTexture(GL_TEXTURE_2D, fb->tex_name));
  GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
  GLCHK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));

  fb->vcsm_info.width = fb_width;
  fb->vcsm_info.height = fb_height;
  fb->egl_fb_image = eglCreateImageKHR(raspitex_state->display, EGL_NO_CONTEXT,
                                 EGL_IMAGE_BRCM_VCSM, &fb->vcsm_info, NULL);
  if (fb->egl_fb_image == EGL_NO_IMAGE_KHR || fb->vcsm_info.vcsm_handle == 0) {
    vcos_log_error("%s: Failed to create EGL VCSM image\n", VCOS_FUNCTION);
    return -1;
  }

  GLCHK(glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, fb->egl_fb_image));

  GLCHK(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, fb->tex_name, 0));
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    vcos_log_error("GL_FRAMEBUFFER is not complete\n");
    return -1;
  }

  GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
  return 0;
}

static int vcsm_square_init(RASPITEX_STATE *raspitex_state) {
  printf("Using VCSM\n");

  int rc = vcsm_init();

  fb_width = next_power_of_two(raspitex_state->width);
  fb_height = next_power_of_two(raspitex_state->height);

  raspitex_state->egl_config_attribs = vcsm_square_egl_config_attribs;
  rc = raspitexutil_gl_init_2_0(raspitex_state);
  if (rc != 0)
    goto end;

  // Shader for drawing the YUV OES texture
  rc = raspitexutil_build_shader_program(&vcsm_square_oes_shader);
  GLCHK(glUseProgram(vcsm_square_oes_shader.program));
  GLCHK(
      glUniform1i(vcsm_square_oes_shader.uniform_locations[0], 0)); // tex unit

  for (int i = 0; i < NUM_FRAMEBUFFERS; i++) {
    rc = init_framebuffer(&framebuffers[i], raspitex_state);
    if (rc != 0)
      goto end;
  }

  GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
  GLCHK(glGenBuffers(1, &quad_vbo));
  GLCHK(glBindBuffer(GL_ARRAY_BUFFER, quad_vbo));
  GLCHK(glBufferData(GL_ARRAY_BUFFER, sizeof(quad_varray), quad_varray,
                     GL_STATIC_DRAW));

  GLCHK(glClearColor(0, 0, 0, 0));
end:
  return rc;
}

static int vcsm_square_redraw(RASPITEX_STATE *raspitex_state) {
  unsigned char *vcsm_buffer = NULL;
  VCSM_CACHE_TYPE_T cache_type;

  current_fb_idx = (current_fb_idx + 1) % NUM_FRAMEBUFFERS;
  raspitex_state->wait_for_vcsm_read_done(current_fb_idx);

  glClearColor(255, 255, 255, 255);

  GLCHK(glBindFramebuffer(GL_FRAMEBUFFER, framebuffers[current_fb_idx].name));
  GLCHK(glViewport(0, 0, raspitex_state->width, raspitex_state->height));
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Fill the viewport with the camFill the viewport with the camera image
  GLCHK(glUseProgram(vcsm_square_oes_shader.program));

  GLCHK(glActiveTexture(GL_TEXTURE0));
  GLCHK(glBindTexture(GL_TEXTURE_EXTERNAL_OES, raspitex_state->texture));

  GLCHK(glBindBuffer(GL_ARRAY_BUFFER, quad_vbo));
  GLCHK(
      glEnableVertexAttribArray(vcsm_square_oes_shader.attribute_locations[0]));
  GLCHK(glVertexAttribPointer(vcsm_square_oes_shader.attribute_locations[0], 2,
                              GL_FLOAT, GL_FALSE, 0, 0));

  double lo[3], up[3];
  raspitex_state->get_thresholds(lo, up);
  GLCHK(glUniform3f(vcsm_square_oes_shader.uniform_locations[1], lo[0], lo[1], lo[2])); // lower thresh
  GLCHK(glUniform3f(vcsm_square_oes_shader.uniform_locations[2], up[0], up[1], up[2])); // lower thresh

  GLCHK(glDrawArrays(GL_TRIANGLES, 0, 6));

  GLCHK(glFinish());

  // Make the buffer CPU addressable with host cache enabled
  vcsm_buffer = (unsigned char *)vcsm_lock_cache(
      framebuffers[current_fb_idx].vcsm_info.vcsm_handle, VCSM_CACHE_TYPE_HOST, &cache_type);
  if (!vcsm_buffer) {
    vcos_log_error("Failed to lock VCSM buffer for handle %d\n",
                  framebuffers[current_fb_idx].vcsm_info.vcsm_handle);
    return -1;
  }

  raspitex_state->enqueue_mat(vcsm_buffer, current_fb_idx, raspitex_state->width, raspitex_state->height, fb_width, fb_height);

  GLCHK(glUseProgram(0));

  return 0;
}

int vcsm_square_open(RASPITEX_STATE *raspitex_state) {
  vcos_log_trace("%s", VCOS_FUNCTION);

  raspitex_state->ops.gl_init = vcsm_square_init;
  raspitex_state->ops.redraw = vcsm_square_redraw;
  raspitex_state->ops.update_texture = raspitexutil_update_texture;
  return 0;
}