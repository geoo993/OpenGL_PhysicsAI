#ifndef EMITTER_H
#define EMITTER_H
/*
 *	Particle system demo - class for a point emitter
 *
 *	This Code Was Created By Jeff Molofee 2000
 *	If You've Found This Code Useful, Please Let Me Know.
 *	Visit My Site At nehe.gamedev.net
 *
 *	Original: Lesson #19: Particle Engine Using Triangle Strips
 *	http://nehe.gamedev.net/tutorial/particle_engine_using_triangle_strips/21001/
 *	Refactored into classes by Ross Paterson.
 */

#include "Vector3.h"
#include "Particle.h"

// A simple point emitter for a particle system
class Emitter {
public:
	static const int NUM_PARTICLES = 1000;

private:
	Vector3	position;	// Initial position of particles
	Vector3	velocity;	// Average initial velocity of particles
	float vel_deviation;

	// Fixed collection of particles, which are re-used when they die.
	Particle particle[NUM_PARTICLES];

public:
	Emitter();

	// Select a particle by index
	// Requires: 0 <= n && n < NUM_PARTICLES
	const Particle &getParticle(int n) const { return particle[n]; }

	// Advance all the emitter's particles in time by delta_t seconds
	// Requires: delta_t >= 0
	void step(float delta_t);

	// Adjust the initial speed of particles
	void speedUp();
	void speedDown();
	void speedRight();
	void speedLeft();
};

#endif
