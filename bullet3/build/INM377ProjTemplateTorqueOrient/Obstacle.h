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
    Obstacle(): centre(btVector3(0,0,0)), radius(0.0){}
    Obstacle(const btVector3 &c, btScalar r) :
    centre(c), radius(r) {}
    
    btVector3 getCentre() const { return centre; };
    btScalar getRadius() const { return radius; };
        
    
    // Returns true if pos is past the obstacle.
	// In that case, the output parameter target is set to a point
	// to aim towards to return to the right side of the obstacle.
    bool missed(const btVector3 &pos, btVector3 &target) const;

	// Test whether a sphere of radius boundingRadius is on a
	// a path that will meet the obstacle.
	// Returns true if this obstacle lies within boundingRadius
	// of a ray starting at pos and proceeding in direction vel.
	// In that case, the output parameters are set:
	// distance = distance from pos to the point of impact
	// avoidPoint = a point on the obstacle that the object
	//	should steer away from.
    bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;

    virtual ~Obstacle();
};

#endif
