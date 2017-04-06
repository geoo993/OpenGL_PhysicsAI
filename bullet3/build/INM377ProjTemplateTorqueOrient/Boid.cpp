//
//  Boid.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "Boid.h"

Boid::Boid(){
    
    m_body = nullptr; 
    m_collShape = nullptr;
    m_hullShape = nullptr;
    
    m_transform = btTransform();
    
    m_hullShape = new btConvexHullShape();
}

Boid::Boid(btRigidBody* body){
    
    m_body = body; 
    m_collShape = nullptr;
    m_hullShape = nullptr;
    
    m_transform = btTransform();
    
    m_hullShape = new btConvexHullShape();
}

Boid::~Boid(){
    delete m_body;
    delete m_collShape;
    delete m_hullShape;
}

void  Boid::Set( const btVector3 &position)
    {

    m_hullShape->addPoint(btVector3(bGet(BoidsValues::BRADIUS), 0, 0));
    m_hullShape->addPoint(btVector3(0, bGet(BoidsValues::BHEIGHT), 0));
    m_hullShape->addPoint(btVector3(0, 0, bGet(BoidsValues::BWIDTH)));
    m_hullShape->addPoint( btVector3(0, 0, -bGet(BoidsValues::BWIDTH)));
    m_collShape = m_hullShape;
    
    //set position
    m_transform.setIdentity();
    m_transform.setOrigin(position);

    //set mass
    btVector3 bLocalInertia;
    m_hullShape->calculateLocalInertia(bGet(BoidsValues::BMASS), bLocalInertia);
    
}

void Boid::Activate(){
    
    m_body->setAnisotropicFriction(m_hullShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    m_body->setFriction(0.5);
    m_body->setLinearVelocity(btVector3(1,0,0));
    m_body->activate(true);
}

btVector3 Boid::Seek(const btVector3 &target) const{
    
    btVector3 desired = target - m_body->getCenterOfMassPosition();
    desired.normalize();
    desired = desired * bGet(BoidsValues::BMAXSPEED); //m_maxSpeed;
    
    btVector3 steer = desired - m_body->getLinearVelocity();
    steer = steer.normalize() * bGet(BoidsValues::BMAXFORCE);
    
    return steer;
}


btVector3 Boid::AvoidanceForce(const std::vector<Obstacle *>& obstacles) const{
    
    btVector3 bposition = btVector3(m_body->getCenterOfMassPosition().x(), 0.0,m_body->getCenterOfMassPosition().z());
    btVector3 oAvoidance = btVector3(0,0,0);
    
    for (unsigned int o = 0; o < obstacles.size(); ++o){
        
        btScalar oRadius = obstacles[o]->getRadius() * 20;
        btVector3 oCenter = obstacles[o]->getCentre();
        
        btScalar aheadDistanceToObstacle = oCenter.distance(bposition);
        btVector3 bright = GetRight();
        btScalar oAvoidanceforce = btDot(bright.normalized() ,oCenter.normalized()) ;
        
        if ((aheadDistanceToObstacle <= oRadius)){
            
            if (oAvoidanceforce < 0){
                //-1 right
                oAvoidance = btVector3( 0, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)), 0);
            }else if(oAvoidanceforce > 0) {
                //1 left
                oAvoidance = btVector3( 0, -(bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)), 0);
            }
            
        } 
        
        
        if (obstacles[o] == nullptr ){
            std::cout << "null actor" << o <<std::endl; 
        }
    }
    
    return oAvoidance;
    
}

btVector3 Boid::LiftForce(const btScalar & boundary) const{
    
    //lift
    if( (boundary - m_body->getCenterOfMassPosition().y())  < 20.0){
        return btVector3(0, -(bGet(Boid::BoidsValues::BLift)), 0);
    }else if( (m_body->getCenterOfMassPosition().y()) < 20.0 ) { 
        return btVector3(0,bGet(Boid::BoidsValues::BLift), 0);
    }else{
        return btVector3(0,0, 0);
    }
    
}



