#ifndef MY_FLOCKING_DEMO_H
#define MY_FLOCKING_DEMO_H

#include "btBulletDynamicsCommon.h"
#include "GlutDemoApplication.h"
#include <vector>

//#include "MyCollisionDemo.h"
#include "Obstacle.h"
#include "Boid.h"

class Flock {

	// declare dummies as private to forbid copying
	Flock(const Flock &flock) {}
	Flock & operator=(const Flock &flock) { return *this; }

    btVector3 CollisionAvoidance(const Boid *actor) const ;
    btVector3 VelocityMarching(const Boid *actor) const;
    btVector3 FlockCentering(const Boid *actor) const;
    
public:
	Flock() {}
    
    btScalar m_borderWidth;
    btScalar m_borderHeight;
    std::vector<Boid*> m_boids;
    std::vector<Obstacle *> m_obstacles;
    
    void CreateFlock(const btScalar &width, const btScalar &height, const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles );

	// Add a boid with the given body.
	// (deletion of the body is handled by the Demo class)
	void addBoid(btRigidBody* b);

	// Add an obstacle for boids to avoid.
	void addObstacle(Obstacle* o);

	// Apply steering forces to each boid in the flock.
	void steer() const;
    
    void Run();
    
    void FlockBoids();
    
    void Update();
    
    void Borders();

    
};

//class MyFlockingDemo : public MyCollisionDemo
//{
//	Flock flock;
//
//	void addSphereObstacle(btSphereShape *shape, const btVector3 &pos);
//	void addColumnObstacle(btCylinderShape *shape, const btVector3 &pos);
//
//public:
//	virtual ~MyFlockingDemo();
//
//	void	initPhysics();
//};

#endif //MY_FLOCKING_DEMO_H
