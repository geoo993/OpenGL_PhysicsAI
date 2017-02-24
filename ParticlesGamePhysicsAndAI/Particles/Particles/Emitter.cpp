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
#include <cstdlib>

#include "Vector3.h"
#include "Particle.h"
#include "Emitter.h"


// random value between -v and v
static float randOffset(float v) {
	return (rand()/float(RAND_MAX)*2 - 1)*v;
}

// random vector in the cube of side 2*v centred on the origin
static Vector3 randVector3(float v) {
	return Vector3(randOffset(v), randOffset(v), randOffset(v));
}

Emitter::Emitter() : vel_deviation(3) {}

// Advance all the emitter's particles in time
void Emitter::step(float delta_t) {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle &p = particle[i];
		if (p.isAlive())
			p.step(delta_t);

		// If particle is burned out, give it new life
		if (! p.isAlive())
			p.restart(position,
				velocity + randVector3(vel_deviation));
	}
}

const float speed_limit = 16;

// Increase upward speed
void Emitter::speedUp() {
	if (velocity.y < speed_limit)
		velocity.y += 0.4f;
}

// Increase downward speed
void Emitter::speedDown() {
	if (velocity.y > -speed_limit)
		velocity.y -= 0.4f;
}

// Increase speed to the right
void Emitter::speedRight() {
	if (velocity.x < speed_limit)
		velocity.x += 0.4f;
}

// Increase speed to the left
void Emitter::speedLeft() {
	if (velocity.x > -speed_limit)
		velocity.x -= 0.4f;
}
