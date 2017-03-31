//
//  Obstacle.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "Obstacle.h"



bool Obstacle::missed(const btVector3 &pos, btVector3 &target) const{
    
    return false;
}

bool Obstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const{
    
    
    return false;
}

Obstacle::~Obstacle(){
    
}
