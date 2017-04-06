#ifndef BOID_H
#define BOID_H

#include "btBulletDynamicsCommon.h"

#include "Obstacle.h"
#include <iostream>
#include <vector>

// A boid is a single agent in the flock
struct Boid {

    enum class BoidsValues 
    {
        BLift, //lift boid and force on the vertical space
        BDRAG,
        BANGULARDRAG,
        
        //A maximum acceleration, expressed as a fraction of the maximum speed, is used to truncate over-anxious requests for acceleration, hence providing for smooth changes of speed and heading
        BMAXSPEED, //boid Maximum speed

        BMAXFORCE, // Maximum steering force
        BMASS,   // mass of each boid
        BRADIUS, // radius of a bounding sphere of the shape
        BWIDTH,
        BHEIGHT,
        BROTATEBACK, //turn boids around 
        BMAXSEEAHEAD,//this ahead vector length defines how far the character will "see". The greater this is, the earlier the character will start acting to dodge an obstacle, because it will perceive it as a threat even if it's far away.
        BMAXAVOIDANCEFORCE,//boid obscacle avoidance voice
    };
    
    btScalar bGet(BoidsValues value) const
    {
        switch (value)
        {
            case BoidsValues::BLift : return 6.0;//15.0;
            case BoidsValues::BDRAG : return 3.0;
            case BoidsValues::BANGULARDRAG : return 5.0;
            case BoidsValues::BROTATEBACK: return 10.0;//2.0;
            case BoidsValues::BMAXSPEED: return 20;//14.0;
            case BoidsValues::BMAXFORCE : return 8.0;//1.0;
            case BoidsValues::BMASS : return 1.0;
            case BoidsValues::BRADIUS : return 4.0;
            case BoidsValues::BWIDTH : return 3.0;
            case BoidsValues::BHEIGHT : return 2.0;
            case BoidsValues::BMAXSEEAHEAD : return 10;
            case BoidsValues::BMAXAVOIDANCEFORCE : return 4.0;//2.0;
        }
    }
    
    btConvexHullShape * m_hullShape;
    
    btCollisionShape *m_collShape;
    
    btTransform m_transform;
   
    Boid(btRigidBody* body);
    Boid();
    ~Boid();
    
    // Body of the boid (deletion managed by the Demo class)
    btRigidBody* m_body;

    //get boid hull shape
    btConvexHullShape* GetHullShape() const { return m_hullShape; };
    //get boid collision shape
    btCollisionShape* GetCollShape() const { return m_collShape; };
    
    //get transform of boid
    btTransform GetTransform() const { return m_transform; }
    
    btVector3 GetPosition() const { return m_body->getCenterOfMassPosition(); }
    
    btMatrix3x3 GetMatOrientation() const { return btMatrix3x3(m_body->getOrientation());} // quat to matrix
    
    btScalar GetMass() const { 
        return m_body->getInvMass();
    };
    btVector3 GetGravity() const { 
        return m_body->getGravity();
    };
    
    // Unit vector for the direction in which the boid is heading
    //Geometric flight is based on incremental translations along the object's "forward direction," its local positive Z axis
    btVector3 GetHeading() const { 
        return btVector3(GetMatOrientation()[0][0], GetMatOrientation()[0][1], GetMatOrientation()[0][2]);
    };
    
    btVector3 GetBack() const { 
        return GetHeading().normalized() * -1.0;
    };
    
    btVector3 GetRight() const { 
        return btVector3(GetMatOrientation()[2][0], GetMatOrientation()[2][1], GetMatOrientation()[2][2]);
    };
    
    btVector3 GetLeft() const { 
        return GetRight().normalized() * -1.0;
    };
    
    btVector3 GetUp() const { 
        return btVector3(GetMatOrientation()[1][0], GetMatOrientation()[1][1], GetMatOrientation()[1][2]);
    };
    
    btVector3 GetDown() const { 
        return GetUp().normalized() * -1.0;
    };
    
    btVector3 GetVelocity() const { return m_body->getLinearVelocity(); }; 
    
    void Set(const btVector3 &position);
    void Activate();
    
    
    // Forces on the boid
    btVector3 Seek(const btVector3 &target) const;
    btVector3 LiftForce(const btScalar & boundary) const;
    btVector3 AvoidanceForce(const std::vector<Obstacle *>& obstacles) const;
    
};

#endif // BOID_H
