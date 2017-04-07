#ifndef BOID_H
#define BOID_H

#include "btBulletDynamicsCommon.h"

#include "Obstacle.h"
#include <iostream>
#include <vector>
#include "Extension.h"

// A boid is a single agent in the flock
struct Boid {
    
    //boids parameters
    enum class BoidsValues 
    {
        BLift, //lift boid and force on the vertical space
        BDRAG,//the linear drag of the boids
        BANGULARDRAG,// the angular drag of the boids
        
        //A maximum acceleration, expressed as a fraction of the maximum speed, is used to truncate over-anxious requests for acceleration, hence providing for smooth changes of speed and heading
        BMAXSPEED, //boid Maximum speed

        BMAXFORCE, // Maximum steering force
        BMASS,   // mass of each boid
        BROTATEBACK, //turn boids around to face the scene  
        BMAXAVOIDANCEFORCE,//boid obscacle avoidance voice
        BBORDERBOUNDARY,// boundary of the scene
    };
    
    //seting the values of boids parameters
    btScalar bGet(BoidsValues value) const
    {
        switch (value)
        {
            case BoidsValues::BLift : return 2.0;//15.0;
            case BoidsValues::BDRAG : return 4.0;//3.0
            case BoidsValues::BANGULARDRAG : return 5.0;
            case BoidsValues::BROTATEBACK: return 10.0;//2.0;
            case BoidsValues::BMAXSPEED: return 25;//14.0;
            case BoidsValues::BMAXFORCE : return 8.0;//1.0;
            case BoidsValues::BMASS : return 1.0;
            case BoidsValues::BMAXAVOIDANCEFORCE : return 4.0;//2.0;
            case BoidsValues::BBORDERBOUNDARY : return 60.0;
        }
    }
    
    //boids hull shape for creating the shape of the boids
    btConvexHullShape * m_hullShape;
    
    //boid transform used for setting position, rotation and scale
    btTransform m_transform;
    
    //boids constructor and destructor
    Boid();
    Boid(btRigidBody* body);
    ~Boid();
    
    //Body of the boid (deletion managed by the Demo class)
    btRigidBody* m_body;
    
    //Set boid position
    void SetPosition(const btVector3 &position);
    
    //activate boid
    void Activate();
    
    //get boid hull shape
    btConvexHullShape* GetHullShape() const { return m_hullShape; };
    
    //get transform of boid
    btTransform GetTransform() const { return m_transform; }
    
    btMatrix3x3 GetMatOrientation() const { return btMatrix3x3(m_body->getOrientation());} // quat to matrix

    // Unit vector for the direction in which the boid is heading
    //Geometric flight is based on incremental translations along the object's "forward direction," its local positive Z axis
    btVector3 GetHeading() const { 
        return btVector3(GetMatOrientation()[0][0], GetMatOrientation()[0][1], GetMatOrientation()[0][2]);
    };
    
    // Unit vector for the BACKWARD direction of the boid
    btVector3 GetBack() const { 
        return GetHeading().normalized() * -1.0;
    };
    // Unit vector for the RIGHT direction of the boid
    btVector3 GetRight() const { 
        return btVector3(GetMatOrientation()[2][0], GetMatOrientation()[2][1], GetMatOrientation()[2][2]);
    };
    // Unit vector for the LEFT direction of the boid
    btVector3 GetLeft() const { 
        return GetRight().normalized() * -1.0;
    };
    // Unit vector for the UP direction of the boid
    btVector3 GetUp() const { 
        return btVector3(GetMatOrientation()[1][0], GetMatOrientation()[1][1], GetMatOrientation()[1][2]);
    };
    // Unit vector for the DOWN direction of the boid
    btVector3 GetDown() const { 
        return GetUp().normalized() * -1.0;
    };
    
    //returns a force that will be used to move a boid toward a desired position, this is used in the cohesion
    btVector3 Seek(const btVector3 &target) const;
    
    //returns a lift force used to make boids rise off the ground
    btVector3 LiftForce(const btScalar & boundary) const;
    
    //returns torque force that will make boids turn away from obstacles 
    btVector3 AvoidanceForce(const std::vector<Obstacle *>& obstacles) const;
    
    //This is used to apply steering torque force to each boid in the flock to turn them back in the scene
    void SteerBack();
    
};

#endif // BOID_H
