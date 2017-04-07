//
//  Obstacle.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "Obstacle.h"

//returns true if this obstacle lies within boundingRadius of an obstacle
bool Obstacle::InPath(const btVector3 &boidPosition) const{
    
    btScalar obstacleRadius = getRadius() * 60;
    btVector3 obstacleCenter = getCentre();
    btScalar aheadDistanceToObstacle = obstacleCenter.distance(boidPosition);
  
    return aheadDistanceToObstacle <= obstacleRadius;
}

//returns the obstacle avoidance force when boid is in path of an obstacle 
btVector3 Obstacle::GetAvoidanceForce(const btScalar &obstacleDirectionAngle, const btScalar &boidLocalAngle, const btScalar &avoidanceForce) const{
    
    btVector3 obstacleAvoidanceForce = btVector3(0,0,0);
    
    //comparing boid angle relative to the obstacle postion and getting the boid to turn right or left 
    if(obstacleDirectionAngle > 0.0 && obstacleDirectionAngle < 90.0 ){
        
        if( boidLocalAngle > 90.0 && boidLocalAngle < 180.0 ){
            //45->-125     then left is 45 to 125 and right is 125 to -125
            bool right = ( (boidLocalAngle > 125.0 && boidLocalAngle < 180.0) || (boidLocalAngle <= -125.0 && boidLocalAngle > -180.0) ) ;
            bool left = (boidLocalAngle > 45.0 && boidLocalAngle <= 125.0);
            if (right){
                obstacleAvoidanceForce = btVector3( 0, -(avoidanceForce),0 );//right
            }
            if (left){
                obstacleAvoidanceForce = btVector3( 0, (avoidanceForce),0 );//left
            }
        }
    }else if(obstacleDirectionAngle >= 90.0 && obstacleDirectionAngle < 180.0 ){
        if( boidLocalAngle > 0.0 && boidLocalAngle < 90.0 ){
            //-45->125     then left is -45 to 45 and right is 45 to 125
            bool right = (boidLocalAngle < 125.0 && boidLocalAngle > 45.0);
            bool left = ((boidLocalAngle <= 45.0 && boidLocalAngle > 0.0) || (boidLocalAngle < 0.0 && boidLocalAngle > -45.0));
            if (right){
                obstacleAvoidanceForce = btVector3( 0, -(avoidanceForce),0 );//right
            }
            if (left){
                obstacleAvoidanceForce = btVector3( 0, (avoidanceForce),0 );//left
            }
        }
        
    }else if(obstacleDirectionAngle < 0.0 && obstacleDirectionAngle > -90.0){
        if( boidLocalAngle < -90.0 && boidLocalAngle > -180.0 ){
            
            //-45->125     then left is -125 to 125 and right is -45 to -125
            bool right = (boidLocalAngle < -45.0 && boidLocalAngle > -125.0);
            bool left = ((boidLocalAngle <= -125.0 && boidLocalAngle > -180.0) || (boidLocalAngle < 180.0 && boidLocalAngle > 125.0));
            
            if (right){
                obstacleAvoidanceForce = btVector3( 0, -(avoidanceForce),0 );//right
            }
            if (left){
                obstacleAvoidanceForce = btVector3( 0, (avoidanceForce),0 );//left
            }
        }
    }else if(obstacleDirectionAngle <= -90.0 && obstacleDirectionAngle > -180.0 ){
        if( boidLocalAngle < 0.0 && boidLocalAngle > -90.0){
            
            //45->-125     then left is -45 to -125 and right is -45 to 45
            bool right = ((boidLocalAngle < 0.0 && boidLocalAngle > -45.0) || (boidLocalAngle < 45.0 && boidLocalAngle > 0.0));
            bool left = (boidLocalAngle <= -45.0 && boidLocalAngle > -125.0);
            
            if (right){
                obstacleAvoidanceForce = btVector3( 0, -(avoidanceForce),0 );//right
            }
            if (left){
                obstacleAvoidanceForce = btVector3( 0, (avoidanceForce),0 );//left
            }
            
        }
    }
    
    return obstacleAvoidanceForce;
}

