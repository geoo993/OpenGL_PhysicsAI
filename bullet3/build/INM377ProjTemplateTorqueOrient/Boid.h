#ifndef BOID_H
#define BOID_H

#include "btBulletDynamicsCommon.h"

#include "Obstacle.h"

#include <vector>

// A boid is a single agent in the flock
class Boid {
	// Body of the boid (deletion managed by the Demo class)
	btRigidBody* body;

	// Unit vector for the direction in which the boid is heading
	btVector3 heading() const;

	// Can the boid see the point?
	bool canSee(const btVector3 &pos) const;

	// Forces on the boid
	btVector3 physicalForce() const;
	btVector3 flockingForce(const std::vector<Boid>& boids) const;
	btVector3 avoidanceForce(const std::vector<Obstacle *>& obstacles) const;

    
//    - (instancetype)initWithName:(char *)name
//    mass:(float)mass   //1
//    convex:(BOOL)convex  //2
//    tag:(int)tag      //3
//    
    
public:
	static btCollisionShape *shape;

	// radius of a bounding sphere of the shape
	static const btScalar radius;

	// mass of each boid
	static const btScalar mass;

    Boid(btRigidBody* b){ body= b; }

	// Apply steering forces (called on each iteration of the physics
	// engine) including physical forces of flight, flocking behaviour
	// and avoiding obstacles.
	void steer(const std::vector<Boid>& boids,
		const std::vector<Obstacle *>& obstacles) const;
};

#endif // BOID_H
