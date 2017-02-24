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
#include <cstdlib>

#include "Vector3.h"
#include "Particle.h"


// Gravitational acceleration at the Earth's surface (downwards), in m/s^2
static const Vector3 gravity(0.0f, -9.8f -20.0f, 0.0f);

// Random float, uniformly distributed between 0 and 1
static float randFloat() {
	return rand()/float(RAND_MAX);
}

// Random age rate, per second (reciprocal of lifetime)
static float randAgeRate() {
	return 1/(randFloat() + 1);
}

// Particles start off invisible.  When they die, they will be restarted
// by the emitter and become visible.
Particle::Particle() : visible(false), life(1.0f), age_rate(randAgeRate()) {}

// Give a particle new life with initial position and velocity
void Particle::restart(const Vector3 &pos, const Vector3 &vel) {
	visible = true;
	position = pos;
	velocity = vel;
	life = 1.0f;
	age_rate = randAgeRate();
}

const double ground_y = -10.;
const double  restitution = 0.75;
const double drag = 0.5;
const Vector3 wind(20.8f+10, 5.4f, 0.2f);

//Advance the particle's state in time by delta_t seconds
void Particle::step(float delta_t) {
	//Vector3 acc = gravity;
    Vector3 acc = (1-life*1.5f)*gravity-drag*(velocity - wind);
	position += velocity * delta_t;	// Move the particle
	velocity += acc * delta_t;	// Take pull into account
	life -= age_rate * delta_t;	// Age the particle
    
    if (position.y <= ground_y) {
        //velocity.y = fabs(velocity.y)*restitution;
        
        //if (position.y > 5) 
        //velocity.y = -fabs(velocity.y)*restitution;
        
        velocity.y *= -restitution;
        position.y = ground_y + (ground_y - position.y);
        position.y = ground_y + restitution*(ground_y - position.y);
        
    }
}

