#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "btBulletDynamicsCommon.h"

// Abstract class for obstacles.
// A obstacle is a convex volume that boids should avoid.
// Often this will coincide with a static rigid body, but
// this is not mandatory.
class Obstacle {
    
    // Solid sphere
    const btVector3 centre;
    const btScalar radius;
        
public:
    
    Obstacle(const btVector3 &c, btScalar r) : centre(c), radius(r) {}
    
    //obstacle center position
    btVector3 getCentre() const { return centre; };
    
    //obstacle radius
    btScalar getRadius() const { return radius; };

	// Test whether a sphere of radius boundingRadius is on a
	// a path that will meet the obstacle.
	// Returns true if this obstacle lies within boundingRadius of an obstacle
    bool InPath(const btVector3 &boidPosition) const;

    //returns the obstacle avoidance force when boid is in path of an obstacle 
    btVector3 GetAvoidanceForce(const btScalar &obstacleDirectionAngle, const btScalar &boidLocalAngle, const btScalar &avoidanceForce) const;
    
    virtual ~Obstacle();
};

#endif
