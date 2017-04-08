//
//  MyFlockingDemo.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "MyFlockingDemo.h"

//boids constructor
Flock::Flock() {}

//boids destructor
Flock::~Flock() {
    for (unsigned int i = 0; i < m_boids.size(); ++i){
        delete m_boids[i];
    }
    
    for (unsigned int i = 0; i < m_obstacles.size(); ++i){
        delete m_obstacles[i];
    }
}

//create flock of boids and obstacles
void Flock::CreateFlock(const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles){
    m_boids = boids; 
    m_obstacles = obstacles;
}

// Add a boid with the given body.
// (deletion of the body is handled by the Demo class)
void Flock::addBoid(btRigidBody* b){
    m_boids.push_back(new Boid(b));
}

// Add an obstacle for boids to avoid.
void Flock::addObstacle(Obstacle* o){
    m_obstacles.push_back(o);
}

//collision avoidance or seperation always maintain prudent separation from their neighbors
btVector3 Flock::CollisionAvoidance(const Boid *actor) const{
    
    //Collision Avoidance: avoid collisions with nearby flockmates
    //Generally, one boid's awareness of another is based on the distance and direction of the offset vector between them.
    //Static collision avoidance and dynamic velocity matching are complementary.
    //Collision avoidance is the urge to steer a way from an imminent impact.
    //Static collision avoidance is based on the relative position of the flockmates and ignores their velocity.
    //Separation/collision avoidaance is the behavior that causes an actor to steer away from all of its neighbors.
    
    btVector3 c(0,0,0);
    int neighborCount = 0;
    const btScalar m_neighborhoodSphericalZone = 20.0;// also known as the neighbor radius
    
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
    return c;
}

//velocity marching or allignment to steer actor to match the direction and speed of neighbors
btVector3 Flock::VelocityMarching(const Boid *actor) const{
    
    //Alignment is a behavior that causes a particular actor to line up with actors close by.
    //the flock quickly becomes "polarized", its members heading in approximately the same direction at approximately the same speed.
    //Velocity Matching: attempt to match velocity with nearby flockmates
    //velocity matching is based only on velocity and ignores position. It is a predictive version of collision avoidance: if the boid does a good job of matching velocity with its neighbors, it is unlikely that it will collide with any of them any time soon. 
    btVector3 v(0,0,0);//or sum
    
    //in a huge flock spread over vast distances, an individual bird must have a localized and filtered perception of the rest of the flock. A bird might be aware of three categories: itself, it's two or three nearest neighbors, and the rest of the flock.
    //those close enough to be considered neighbors of the specified actor
    int neighborCount = 0;
    //The neighborhood is defined as a spherical zone of sensitivity centered at the boid's local origin.
    const btScalar m_neighborhoodSphericalZone = 40.0;// also known as the neighbor radius
    
    //If an actor is found within the radius, its velocity is added to the computation vector, and the neighbor count is incremented.
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

//boids stay near one another (flock centering or cohesion)
btVector3 Flock::FlockCentering(const Boid *actor) const{
    
    //Flock Centering: attempt to stay close to nearby flockmates
    //Flock centering makes a boid want to be near the center of the flock.
    //Cohesion is a behavior that causes actor to steer towards the "center of mass" - that is, the average position of the actors within a certain radius.
    
    // cohesion behavior
    // return a vector that will steer our current velocity towards the center of mass of all nearby neighbors
    btVector3 p(0,0,0);//or sum
    int neighborCount = 0;
    btScalar m_neighborhoodSphericalZone = 40.0;// also known as the neighbor radius
    
     //When a neighboring actor is found, the distance from the agent to the neighbor is added to the computation vector.
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            // get all my nearby neighbors inside the Spherical Zone/radius of this current actor
            if (distance > 0 && distance < (m_neighborhoodSphericalZone)){
                
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
    
    //the computation vector is divided by the neighbor count, resulting in the position that corresponds to the center of mass. However, we don't want the center of mass itself, we want the direction towards the center of mass, so we recompute the vector as the distance from the actor to the center of mass. 
    p = btVector3( p.x() / btScalar(neighborCount), p.y() / btScalar(neighborCount), p.z() / btScalar(neighborCount));
    p = actor->Seek(p);
    
    return p;
}


//update the flocking boids
void Flock::UpdateFlock(){
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        
        //The computational abstraction that combines process, procedure, and state is called an actor
        Boid* actor = m_boids[b];
        btRigidBody* bbody = actor->m_body;
        
        // separation behavior to steer in the oposite direction from each of our nearby neigbhors
        btVector3 separation = CollisionAvoidance(actor);
        
        //alignement behavior that causes a particular actor to line up with actors close by.
        btVector3 alignment = VelocityMarching(actor);
        
        //cohesion behavior: attempts to stay close to nearby flockmates
        btVector3 cohesion = FlockCentering(actor);
        
        //combine all behaviours
        btVector3 combinedbehaviours = (alignment * 2.0) + cohesion + separation;
        combinedbehaviours.safeNormalize();//can safe normalise
        
        //add a force when the combined flocking behaviors is zero, to keep the boid moving
        if (combinedbehaviours.x()==0 && combinedbehaviours.y()==0&&combinedbehaviours.z()==0){
            combinedbehaviours = btVector3(1.0,0.0,1.0);
        }
        
        btScalar bmass = bbody->getInvMass();
        btVector3 bgravity = bbody->getGravity();
        btTransform btransform = btTransform(bbody->getOrientation());
        btVector3 worldup(0, 1, 0);
        btVector3 btop = actor->GetUp();
        btVector3 bfront = btransform * btVector3(1, 0, 0);
        btVector3 bdirection = actor->GetHeading();
        btVector3 bthrust = actor->bGet(Boid::BoidsValues::BMAXSPEED) * bfront;
        btVector3 bdrag = -(actor->bGet(Boid::BoidsValues::BDRAG)) * bbody->getLinearVelocity();
        btVector3 bangulardrag =  -(actor->bGet(Boid::BoidsValues::BANGULARDRAG)) * bbody->getAngularVelocity();
        btVector3 blift = actor->LiftForce(actor->bGet(Boid::BoidsValues::BBORDERBOUNDARY)) - bgravity ;
        
        //Applying linear force to move the boids
        bbody->applyCentralForce((bthrust + combinedbehaviours + blift + bgravity + bdrag) * bmass);
        
        //This aligns the boid’s orientation to bdir with some drag torque (depending on the angular velocity)
        bbody->applyTorque((2.0 * bfront.cross(bdirection) + bangulardrag) * bmass);//using F = m * a function
        
        //This turns the boid around the vertical axis
        bbody->applyTorque((-0.5 * worldup) * bmass);//using F = m * a function
        
        //This flattens the boid (with some drag torque)
        bbody->applyTorque( (0.5 * btop.cross(worldup) + bangulardrag) * bmass);//using F = m * a function
        
        //This makes the boid avoid obsatcles
        bbody->applyTorque( (bangulardrag + actor->AvoidanceForce(m_obstacles)) * bmass );//using F = m * a function
        
        //This steers the boids back around, so they never go off the scene
        if (actor->SteerBack() == true){
            btVector3 steer(0, actor->bGet(Boid::BoidsValues::BROTATEBACK), 0);
            bbody->applyTorque((steer) * bmass); //using F = m * a function
        }
    }
    
    
}


