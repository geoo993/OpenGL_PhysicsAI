/*
 *	Particle system demo - OpenGL display
 *
 *	This Code Was Created By Jeff Molofee 2000
 *	If You've Found This Code Useful, Please Let Me Know.
 *	Visit My Site At nehe.gamedev.net
 *
 *	Original: Lesson #19: Particle Engine Using Triangle Strips
 *	http://nehe.gamedev.net/tutorial/particle_engine_using_triangle_strips/21001/
 *	Refactored by Ross Paterson
 */
//#ifdef _WIN32
//# include <windows.h>
//#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include "Vector3.h"
#include "Emitter.h"
#include "DrawGLScene.h"

static const GLfloat colour[3] = {1.0f,0.5f,0.5f};

static const int bitmap_width = 32;
static const int bitmap_height = 32;
static const int bitmap_size = bitmap_width * bitmap_height * 3;

// intensity (0..255) of pixel (x,y) of a fuzzy dot
unsigned int fuzzy_dot(int x, int y) {
	float dx = x - float(bitmap_width+1)/2;
	float dy = y - float(bitmap_height+1)/2;
	float d2 = dx*dx + dy*dy;
	return (unsigned int)(255*exp(-d2/40));
}

// All Setup For OpenGL Goes Here
bool InitGL() {
	// Make a fuzzy dot bitmap
	GLubyte bitmap[bitmap_size];
	for (int i = 0; i < bitmap_width; i++) {
		for (int j = 0; j < bitmap_height; j++) {
			GLubyte *p = &bitmap[3*(i*bitmap_width + j)];
			p[0] = p[1] = p[2] = fuzzy_dot(i, j);
		}
	}

	// Create a texture from the bitmap
	GLuint	texture[1];				// Storage For Our Particle Texture
	glGenTextures(1, &texture[0]);			// Create One Texture
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, bitmap_width, bitmap_height,
		0, GL_RGB, GL_UNSIGNED_BYTE, bitmap);

	glShadeModel(GL_SMOOTH);			// Enable Smooth Shading
	glClearColor(0.0f,0.0f,0.0f,0.0f);		// Black Background
	glClearDepth(1.0f);				// Depth Buffer Setup
	glDisable(GL_DEPTH_TEST);			// Disable Depth Testing
	glEnable(GL_BLEND);				// Enable Blending
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);		// Type Of Blending To Perform
	glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);	// Really Nice Perspective Calculations
	glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);		// Really Nice Point Smoothing
	glEnable(GL_TEXTURE_2D);			// Enable Texture Mapping

	return true;
}

void ReSizeGLScene(GLsizei width, GLsizei height) {	// Resize And Initialize The GL Window
	if (height == 0)				// Prevent A Divide By Zero By
		height = 1;				// Making Height Equal One

	glViewport(0,0,width,height);			// Reset The Current Viewport

    float ratio;
    ratio = width / (float) height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
	glMatrixMode(GL_PROJECTION);			// Select The Projection Matrix
	glLoadIdentity();				// Reset The Projection Matrix

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,200.0f);

	glMatrixMode(GL_MODELVIEW);			// Select The Modelview Matrix
	glLoadIdentity();				// Reset The Modelview Matrix
}

void DrawGLScene(const Emitter &emitter, float zoom) {	// Here's Where We Do All The Drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
	glLoadIdentity();					// Reset The ModelView Matrix
    
	for (GLuint i = 0; i < Emitter::NUM_PARTICLES; i++) {	// Loop Through All The Particles
		const Particle &p = emitter.getParticle(i);
		if (p.isVisible() && p.isAlive()) {
			Vector3 pos = p.getPosition();
			pos.z += zoom;

			// Draw the particle using our colour, faded based on its life
			glColor4f(colour[0],colour[1],colour[2],p.getLife());

			glBegin(GL_TRIANGLE_STRIP);				// Build Quad From A Triangle Strip
				glTexCoord2d(1,1); glVertex3f(pos.x+0.5f,pos.y+0.5f,pos.z); // Top Right
				glTexCoord2d(0,1); glVertex3f(pos.x-0.5f,pos.y+0.5f,pos.z); // Top Left
				glTexCoord2d(1,0); glVertex3f(pos.x+0.5f,pos.y-0.5f,pos.z); // Bottom Right
				glTexCoord2d(0,0); glVertex3f(pos.x-0.5f,pos.y-0.5f,pos.z); // Bottom Left
			glEnd();						// Done Building Triangle Strip
		}
    }
}
