#ifndef BOID_H
#define BOID_H

#include "btBulletDynamicsCommon.h"

#include "Obstacle.h"

#include <vector>

// A boid is a single agent in the flock
class Boid {

    
    btConvexHullShape * m_hullShape;
    
    btCollisionShape *m_collShape;
    
    // radius of a bounding sphere of the shape
    btScalar m_radius;
    
    // mass of each boid
    btScalar m_mass;
    
    btTransform m_trans;
    
    //Geometric flight is based on incremental translations along the object's "forward direction," its local positive Z axis
    btVector3 m_forward;
    
    //Velocity is a vector quantity, referring to the combination of heading (forward) and speed.
    //The magnitude of the turning acceleration varies directly with the object's velocity and with the curvature of its path
    btVector3 m_velocity; 
    
    btVector3 m_acceleration;
    
    btScalar m_maxForce; // Maximum steering force
    
    //A maximum acceleration, expressed as a fraction of the maximum speed, is used to truncate over-anxious requests for acceleration, hence providing for smooth changes of speed and heading
    btScalar m_maxSpeed; //Maximum speed
    
    //between 0 and 1, strenght in which a boid wants to participate in a flock small or strong
    btScalar m_eagerness; 

	// Can the boid see the point?
	bool canSee(const btVector3 &pos) const;

	// Forces on the boid
	btVector3 physicalForce() const;
	btVector3 flockingForce(const std::vector<Boid>& boids) const;
	btVector3 avoidanceForce(const std::vector<Obstacle *>& obstacles) const;

    void applyForce(btVector3 force);
  
    
public:
    
    //Boid(btRigidBody* b);
    Boid();
    ~Boid();
    
    // Body of the boid (deletion managed by the Demo class)
    btRigidBody* m_body;

    //get boid hull shape
    btConvexHullShape* GetHullShape() const { return m_hullShape; };
    //get boid collision shape
    btCollisionShape* GetCollShape() const { return m_collShape; };
    //get radius of boid
    btScalar GetRadius() const { return m_radius; }
    // get mass of  boid
    btScalar GetMass() const { return m_mass; }
    //get transform of boid
    btTransform GetTrans() const { return m_trans; }
    
    // Unit vector for the direction in which the boid is heading
    btVector3 heading() const { return m_forward; };
    
    btVector3 GetVelocity() const { return m_velocity; }; 
    
    
    void Set(const std::vector<btVector3> &shape, const btVector3 &position, const btVector3 &velocity, const btVector3 &acceleration, const btScalar &radius, const btScalar &mass, const btScalar &force, const btScalar &speed);
    void Activate();

};

#endif // BOID_H
