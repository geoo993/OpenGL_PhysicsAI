//
//  Boid.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "Boid.h"


btVector3 Boid::heading() const{
    
    return btVector3(0,0,0);
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
