#ifndef MY_FLOCKING_DEMO_H
#define MY_FLOCKING_DEMO_H

#include "btBulletDynamicsCommon.h"
#include "GlutDemoApplication.h"

#include <vector>

#include "Obstacle.h"
#include "Boid.h"
#include "Extension.h"

class Flock {

	// declare dummies as private to forbid copying
	Flock(const Flock &flock) {}
	Flock & operator=(const Flock &flock) { return *this; }

    btVector3 CollisionAvoidance(const Boid *actor) const ;
    btVector3 VelocityMarching(const Boid *actor) const;
    btVector3 FlockCentering(const Boid *actor) const;
    
public:
    Flock() {}
    
    //btScalar m_borderboundary;
    std::vector<Boid*> m_boids;
    std::vector<Obstacle *> m_obstacles;
    
    //create flock of boids and obstacles
    void CreateFlock(const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles );

	// Add a boid with the given body.
	// (deletion of the body is handled by the Demo class)
	void addBoid(btRigidBody* b);

	// Add an obstacle for boids to avoid.
	void addObstacle(Obstacle* o);
    
    //update the flocking
    void UpdateFlock();

    
};


#endif //MY_FLOCKING_DEMO_H
