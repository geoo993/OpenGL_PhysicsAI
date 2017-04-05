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
    m_velocity = btVector3(0,0,0);
    m_acceleration = btVector3(0,0,0);
    
    m_hullShape = new btConvexHullShape();
}

Boid::~Boid(){
    delete m_body;
    delete m_collShape;
    delete m_hullShape;
}

void  Boid::Set( 
    const btVector3 &position, 
    const btVector3 &velocity, 
    const btVector3 &acceleration)
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
    
    //velocity
    m_velocity = velocity;
    
    //acceleration
    m_acceleration = m_acceleration;
    
}

void Boid::Activate(){
    
    m_body->setAnisotropicFriction(m_hullShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    m_body->setFriction(0.5);
    //m_body->setLinearVelocity(m_velocity);
    m_body->activate(true);
}

void Boid::applyForce(btVector3 force) {
    // We could add mass here if we want A = F / M
    m_acceleration = m_acceleration + force;
}


// Can the boid see the point?
bool Boid::canSee(const btVector3 &pos) const{
    
    return false;
}

btVector3 Boid::Seek(const btVector3 &target) const{
    
    btVector3 desired = target - m_body->getCenterOfMassPosition();
    desired.normalize();
    desired = desired * bGet(BoidsValues::BMAXSPEED); //m_maxSpeed;
    
    btVector3 steer = desired - m_velocity;
    steer = steer.normalize() * bGet(BoidsValues::BMAXFORCE);
    
    return steer;
}

// Forces on the boid
btVector3 Boid::physicalForce() const{
    return btVector3(0,0,0);
}
btVector3 Boid::flockingForce(const std::vector<Boid>& boids) const{
    
    return btVector3(0,0,0);
}
btVector3 Boid::avoidanceForce(const std::vector<Obstacle *>& obstacles) const{
    
    return btVector3(0,0,0);
}





