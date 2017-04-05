//
//  MyFlockingDemo.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "MyFlockingDemo.h"

void Flock::CreateFlock(const btScalar &boundary, const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles){
    m_boids = boids; 
    m_obstacles = obstacles;
    m_borderboundary = boundary;
}

void Flock::addBoid(btRigidBody* b){
    m_boids.push_back(new Boid(b));
}

// Add an obstacle for boids to avoid.
void Flock::addObstacle(Obstacle* o){
    m_obstacles.push_back(o);
}

//output, computing new force
btVector3 Flock::CollisionAvoidance(const Boid *actor) const{
    
    
    //always maintain prudent separation from their neighbors (collision avoidance/seperation)
    //Collision Avoidance: avoid collisions with nearby flockmates
    //Generally, one boid's awareness of another is based on the distance and direction of the offset vector between them.
    //Static collision avoidance and dynamic velocity matching are complementary.
    //Collision avoidance is the urge to steer a way from an imminent impact.
    //Static collision avoidance is based on the relative position of the flockmates and ignores their velocity.
    
    //Separation is the behavior that causes an agent to steer away from all of its neighbors.
    
    // separation behavior
    // steer in the oposite direction from each of our nearby neigbhors
    
    btVector3 c(0,0,0); // or steer
    int neighborCount = 0;
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            // get all my nearby neighbors inside the Spherical Zone/radius of this current actor
            if (distance > 0 && distance < m_neighborhoodSphericalZone){
                
                btScalar tempX = actor->m_body->getCenterOfMassPosition().x() - otherActor->getCenterOfMassPosition().x() ;
                btScalar tempY = actor->m_body->getCenterOfMassPosition().y() - otherActor->getCenterOfMassPosition().y();
                btScalar tempZ = actor->m_body->getCenterOfMassPosition().z() - otherActor->getCenterOfMassPosition().z();
                
                btVector3 difference(tempX, tempY, tempZ);
                difference.normalize();
                
                difference = difference / distance;
                
                c = c + difference;
                
                neighborCount++;
            }
            
        }
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return c;
    }
    
    //The computation vector is divided by the corresponding neighbor count, but before normalizing, there is one more crucial step involved. The computed vector needs to be negated in order for the agent to steer away from its neighbors properly.
    if (neighborCount > 0){
        c = btVector3( c.x() / btScalar(neighborCount), c.y() / btScalar(neighborCount), c.z() / btScalar(neighborCount) );
    }
    
    if (c.length() > 0){
        
        c.normalize();
        c = c * actor->bGet(Boid::BoidsValues::BMAXSPEED);
        c = c - actor->m_body->getLinearVelocity();
        c = c.normalize() * actor->bGet(Boid::BoidsValues::BMAXFORCE); //limit
    }
    
    //c *= -1;
   
    
    return c;
}



btVector3 Flock::VelocityMarching(const Boid *actor) const{
    
    // alignment behavior
    // steer agent to match the direction and speed of neighbors
    
 
    //Alignment is a behavior that causes a particular agent to line up with agents close by.
    //the flock quickly becomes "polarized", its members heading in approximately the same direction at approximately the same speed (velocity marching or allignment)
    //Velocity Matching: attempt to match velocity with nearby flockmates
    //velocity matching is based only on velocity and ignores position. It is a predictive version of collision avoidance: if the boid does a good job of matching velocity with its neighbors, it is unlikely that it will collide with any of them any time soon. 
    btVector3 v(0,0,0);//or sum
    
    //in a huge flock spread over vast distances, an individual bird must have a localized and filtered perception of the rest of the flock. A bird might be aware of three categories: itself, it's two or three nearest neighbors, and the rest of the flock.
    //those close enough to be considered neighbors of the specified actor
    int neighborCount = 0;
    //The neighborhood is defined as a spherical zone of sensitivity centered at the boid's local origin.
    
    
    //If an agent is found within the radius, its velocity is added to the computation vector, and the neighbor count is incremented.
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            // get all my nearby neighbors inside the Spherical Zone/radius of this current actor
            if (distance > 0 && distance < m_neighborhoodSphericalZone){
                btScalar tempX = v.x() + otherActor->getLinearVelocity().x();
                btScalar tempY = v.y() + otherActor->getLinearVelocity().y();
                btScalar tempZ = v.z() + otherActor->getLinearVelocity().z();
                v = btVector3(tempX, tempY, tempZ);
                neighborCount++;
            }
            
        }
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return btVector3(0,0,0);
    }
    
    //Finally, we divide the computation vector by the neighbor count and normalize it (divide it by its length to get a vector of length 1), obtaining the final resultant vector.
    v = btVector3( v.x() / btScalar(neighborCount), v.y() / btScalar(neighborCount), v.z() / btScalar(neighborCount));
    v.normalize();
    v = v * actor->bGet(Boid::BoidsValues::BMAXSPEED);
    
    btVector3 steer = v - actor->m_body->getLinearVelocity();
    steer = steer.normalize() * actor->bGet(Boid::BoidsValues::BMAXFORCE);
    
    return steer;
    
}

btVector3 Flock::FlockCentering(const Boid *actor) const{
    
    //boids stay near one another (flock centering or cohesion)
    //Flock Centering: attempt to stay close to nearby flockmates
    //Flock centering makes a boid want to be near the center of the flock.
    //Cohesion is a behavior that causes agents to steer towards the "center of mass" - that is, the average position of the agents within a certain radius.
    
    // cohesion behavior
    // return a vector that will steer our curent velocity
    // towards the center of mass of all nearby neighbors
    btVector3 p(0,0,0);//or sum
    int neighborCount = 0;
    
     //When a neighboring actor is found, the distance from the agent to the neighbor is added to the computation vector.
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            // get all my nearby neighbors inside the Spherical Zone/radius of this current actor
            if (distance > 0 && distance < (m_neighborhoodSphericalZone * 2.0)){
                
                // find the center of mass of all neighbors
                btScalar tempX = p.x() + otherActor->getCenterOfMassPosition().x();
                btScalar tempY = p.y() + otherActor->getCenterOfMassPosition().y();
                btScalar tempZ = p.z() + otherActor->getCenterOfMassPosition().z();
                
                p = btVector3(tempX, tempY, tempZ);
                neighborCount++;
            }
            
        }
        
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return btVector3(0,0,0) ;
    }
    
    //the computation vector is divided by the neighbor count, resulting in the position that corresponds to the center of mass. However, we don't want the center of mass itself, we want the direction towards the center of mass, so we recompute the vector as the distance from the agent to the center of mass. Finally, this value is normalized and returned.
    p = btVector3( p.x() / btScalar(neighborCount), p.y() / btScalar(neighborCount), p.z() / btScalar(neighborCount));
    //btVector3 tempP = btVector3(
    //                            p.x() - actor->m_body->getCenterOfMassPosition().x(), 
    //                            p.y() - actor->m_body->getCenterOfMassPosition().y(), 
    //                            p.z() - actor->m_body->getCenterOfMassPosition().z());
    //tempP.normalize();
    
    return actor->Seek(p);
}




// Apply steering forces to each boid in the flock.
void Flock::Steer(Boid *actor) const {
    btRigidBody *bbody = actor->m_body;
    
    
    /////////////////////////
    ///////////////-90/////////
    ////////////   /   ////////
    /////////      /     /////
    //////         /        /////
    ////           /           ////
    ///            /            ////-180
    //0       //////////         ///
    ///            /             ///180
    ////           /            ////
    //////         /           ////
    ///////        /         //////
    //////////     /        ///////
    //////////////     ///////
    ////////////////90//////
    btScalar angleFromWorld = Extension::getAngleBetweenTwoPoints(bbody->getCenterOfMassPosition().x()
                                                                  ,0,bbody->getCenterOfMassPosition().z(),
                                                                  0.0,0.0,0.0);
    btScalar angleFromLocal = Extension::getAngleBetweenTwoPoints(actor->GetHeading().x()
                                                                  ,0,actor->GetHeading().z(),
                                                                  0.0,0.0,0.0);
        if (bbody->getCenterOfMassPosition().length() > m_borderboundary){
            
            btVector3 steer(0, actor->bGet(Boid::BoidsValues::BROTATEBACK), 0);
            
            if(angleFromWorld > 0.0 && angleFromWorld < 90.0 ){
                if( (angleFromLocal > 0.0 && angleFromLocal < 45.0) || (angleFromLocal <= 0.0 && angleFromLocal > -125.0) ){
                    bbody->applyTorque(steer);
                }
            }else if(angleFromWorld >= 90.0 && angleFromWorld < 180.0 ){
                if( (angleFromLocal > 125.0 && angleFromLocal < 180.0) || (angleFromLocal <= -45.0 && angleFromLocal > -180.0) ){
                    bbody->applyTorque(steer);
                }
            }else if(angleFromWorld < 0.0 && angleFromWorld > -90.0){
                if( (angleFromLocal > 0.0 && angleFromLocal < 125.0) || (angleFromLocal <= 0.0 && angleFromLocal > -45.0) ){
                    bbody->applyTorque(steer);
                }
            }else if(angleFromWorld <= -90.0 && angleFromWorld > -180.0 ){
                if( (angleFromLocal > 45.0 && angleFromLocal < 180.0) || (angleFromLocal <= -125.0 && angleFromLocal > -180.0) ){
                    bbody->applyTorque(steer);
                }
            }
        }
    
}




void Flock::UpdateFlock(){
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        
        //The computational abstraction that combines process, procedure, and state is called an actor
        Boid* actor = m_boids[b];
        btRigidBody* bbody = actor->m_body;
        btTransform btransform = bbody->getWorldTransform();
        
        btVector3 separation = CollisionAvoidance(actor);//seperation
        btVector3 alignment = VelocityMarching(actor);//alignement
        btVector3 cohesion = FlockCentering(actor);//cohesion
        btVector3 tempVel = alignment + cohesion + separation;
        tempVel.normalize();
        

        btVector3 bthrust =  btVector3((btransform * btVector3(actor->bGet(Boid::BoidsValues::BMAXSPEED), 0, 0)) - btransform.getOrigin());
        btVector3 bdrag = -(actor->bGet(Boid::BoidsValues::BDRAG)) * bbody->getLinearVelocity();//bvelocity;
        btVector3 bangulardrag = -(actor->bGet(Boid::BoidsValues::BANGULARDRAG)) * bbody->getAngularVelocity();//bavelocity
        
        btScalar bdistanceYp = m_borderboundary - actor->m_body->getCenterOfMassPosition().y();
        btScalar bdistanceYn = actor->m_body->getCenterOfMassPosition().y();
       // btVector3 blift = btVector3(0,0,0);
        if( bdistanceYp < 10.0){
            //blift = btVector3(0, -(actor->bGet(Boid::BoidsValues::BLift)), 0) ;
            bbody->applyCentralForce(btVector3(0, -(actor->bGet(Boid::BoidsValues::BLift)), 0));
        }else if( bdistanceYn < 10.0 ) { 
            //blift = btVector3(0,actor->bGet(Boid::BoidsValues::BLift), 0);
            bbody->applyCentralForce(btVector3(0,actor->bGet(Boid::BoidsValues::BLift), 0));
        }
        
        
        btVector3 bgravity = actor->m_body->getGravity();
        bbody->applyCentralForce(bgravity);
        
        //thrust
        bbody->applyCentralForce(bthrust + tempVel); //+ blift;
        
        bbody->applyCentralForce(bdrag);
        bbody->applyTorque(bangulardrag + actor->AvoidanceForce(m_obstacles));
        
        //boundary
        Steer(actor);
    }

    
}


