#ifndef PARTICLE_H
#define PARTICLE_H
/*
 *	Particle system demo - class for a single particle
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

class Particle {
	bool	visible;	// The particle should be shown
	float	life;		// Starts at 1 and decreases to 0
	float	age_rate;	// Rate of decrease in life per second
	Vector3 position;	// Position in meters
	Vector3 velocity;	// Velocity in meters per second

public:
	Particle();

	// Selectors
	bool isVisible() const { return visible; }
	bool isAlive() const { return life > 0; }
	float getLife() const { return life; }
	const Vector3 &getPosition() const { return position; }

	// Give a particle new life with initial velocity and position
	void restart(const Vector3 &pos, const Vector3 &vel);

	// Advance the particle's state in time by delta_t seconds
	// Requires: delta_t >= 0
	void step(float delta_t);
};

#endif
