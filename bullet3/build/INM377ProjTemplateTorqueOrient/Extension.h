//
//  Extension.h
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 29/03/2017.
//
//

#ifndef Extension_h
#define Extension_h

#include "btBulletDynamicsCommon.h"
#include <iostream>

class Extension {
    
public:
    
    static btScalar randFloat(){
        return btScalar( (double)rand() / (RAND_MAX + 1.0));
        //return  static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    
    //Random between 2 floats 
    static btScalar  randomFloatBetween(btScalar min, btScalar max)    
    {    
        return (min + 1) + (((btScalar) rand()) / (btScalar) RAND_MAX) * (max - (min + 1));    
    }
    
    //Random between 2 int 
    static int    randomInt(int min, int max)    
    {    
        return rand() % (max - min) + min + 1;     
    }
    
    static btScalar distance(const btVector3 &a, const btVector3 &b){
        float ax = a.x();
        float ay = a.y();
        float az = a.z();
        
        float bx = b.x();
        float by = b.y();
        float bz = b.x();
        
        btScalar distance = sqrt( pow( (ax - bx), 2.0) + pow( (ay - by), 2.0) + pow( (az - bz), 2.0) );
        
        return distance;
    }
    
};

#endif /* Extension_h */
