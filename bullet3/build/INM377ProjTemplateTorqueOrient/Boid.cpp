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
    
    m_radius = 0.0;
    m_mass = 0.0;
    m_trans = btTransform();
    
    m_hullShape = new btConvexHullShape();
}

Boid::~Boid(){
    delete m_body;
    delete m_collShape;
    delete m_hullShape;
}

void  Boid::Set(const std::vector<btVector3> &shape, const btVector3 &position, const btScalar &radius, const btScalar &mass){
    
    for (unsigned int i = 0; i < shape.size(); ++i){
        m_hullShape->addPoint(shape[i]);
    }
    m_collShape = m_hullShape;
    
    //set position
    m_trans.setIdentity();
    m_trans.setOrigin(position);

    
    //set mass
    m_mass = mass;
    btVector3 bLocalInertia;
    m_hullShape->calculateLocalInertia(m_mass, bLocalInertia);

    
}


// Can the boid see the point?
bool Boid::canSee(const btVector3 &pos) const{
    
    return false;
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
