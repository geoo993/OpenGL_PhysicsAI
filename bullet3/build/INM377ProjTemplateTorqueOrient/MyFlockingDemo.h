#ifndef MY_FLOCKING_DEMO_H
#define MY_FLOCKING_DEMO_H

#include "Boid.h"

class Flock {
    
    //collision avoidance or seperation to always maintain prudent separation from their neighbors
    btVector3 CollisionAvoidance(const Boid *actor) const ;
     
    //velocity marching or allignment to steer actor to match the direction and speed of neighbors
    btVector3 VelocityMarching(const Boid *actor) const;
    
    //boids stay near one another (flock centering or cohesion)
    btVector3 FlockCentering(const Boid *actor) const;
    
public:
    
    //flock constructor
    Flock();
    ~Flock();
    
    std::vector<Boid*> m_boids;
    std::vector<Obstacle *> m_obstacles;
    
    //create flock of boids and obstacles
    void CreateFlock(const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles );

	// Add a boid with the given body.
	// (deletion of the body is handled by the Demo class)
	void addBoid(btRigidBody* b);

	// Add an obstacle for boids to avoid.
	void addObstacle(Obstacle* o);
    
    //update the flocking boids
    void UpdateFlock();

    
};


#endif //MY_FLOCKING_DEMO_H
