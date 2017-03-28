//
//  MiscObstacles.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "MiscObstacles.h"

bool BowlObstacle::missed(const btVector3 &pos, btVector3 &target) const{
    
    return false;
}

bool BowlObstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const{
    
    return false;
}


bool SphereObstacle::missed(const btVector3 &pos, btVector3 &target) const{
    
    return false;
}

bool SphereObstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const{
    
    
    return false;
}



bool ColumnObstacle::missed(const btVector3 &pos, btVector3 &target) const{
    
     return false;
}

bool ColumnObstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const{
    
     return false;
}


bool PlaneObstacle::missed(const btVector3 &pos, btVector3 &target) const{
    
     return false;
}

 bool PlaneObstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const{
     
     
    return false;
}
