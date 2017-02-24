#ifndef DRAWGLSCENE_H
#define DRAWGLSCENE_H
/*
 *      Particle system demo - OpenGL display
 */

#include <GL/gl.h>

#include "Emitter.h"

// All Setup For OpenGL Goes Here
extern bool InitGL();

// Resize And Initialize The GL Window
extern void ReSizeGLScene(GLsizei width, GLsizei height);

// Here's Where We Do All The Drawing
extern void DrawGLScene(const Emitter &emitter, float zoom);

#endif
