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

class Extension {
    
public:
    
    static btScalar getAngleBetweenTwoPoints (btScalar x1,btScalar y1,btScalar z1,btScalar x2,btScalar y2,btScalar z2)
    {
        btScalar theta = btAtan2(z1-z2,x1-x2);
        return -theta*180/3.1415926;
    }
    
    static btScalar randFloat(){
        return btScalar( (double)rand() / (RAND_MAX + 1.0));
        //return  static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    
    //Random between 2 floats 
    static btScalar  randomFloatBetween(btScalar min, btScalar max)    
    {    
        return (min + 1) + (((btScalar) rand()) / (btScalar) RAND_MAX) * (max - (min + 1));    
    }

    static btScalar percentageWith(btScalar value, btScalar min, btScalar max) 
    {
        btScalar difference = (min < 0.0) ? max : max - min;
        return (btScalar(100.0) * ((value - min) / difference));
    }
   
    
};

#endif /* Extension_h */
