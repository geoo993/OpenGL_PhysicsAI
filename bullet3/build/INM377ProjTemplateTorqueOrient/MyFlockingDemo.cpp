//
//  MyFlockingDemo.cpp
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 28/03/2017.
//
//

#include "MyFlockingDemo.h"

void Flock::CreateFlock(const btScalar &width, const btScalar &height,const std::vector<Boid*> boids, const std::vector<Obstacle *> obstacles){
    m_boids = boids; 
    m_obstacles = obstacles;
}

void Flock::addBoid(btRigidBody* b){
    
}

// Add an obstacle for boids to avoid.
void Flock::addObstacle(Obstacle* o){
    
}

// Apply steering forces to each boid in the flock.
void Flock::steer() const{
    
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
    
    btVector3 c(0,0,0); // or steer
    int neighborCount = 0;
    btScalar neighborhoodSphericalZone = 20.0;// alos known as the neighbor radius
    
    //When a neighboring agent is found, the distance from the agent to the neighbor is added to the computation vector.
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            
            if (distance > 0 && distance < neighborhoodSphericalZone){
                
                
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
        c = c * actor->m_maxSpeed;
        c = c - actor->m_velocity;
        c = c.normalize() * actor->m_maxForce; //limit
    }
    
    //c *= -1;
   
    
    return c;
}



btVector3 Flock::VelocityMarching(const Boid *actor) const{
    
    //Alignment is a behavior that causes a particular agent to line up with agents close by.
    //the flock quickly becomes "polarized", its members heading in approximately the same direction at approximately the same speed (velocity marching or allignment)
    //Velocity Matching: attempt to match velocity with nearby flockmates
    //velocity matching is based only on velocity and ignores position. It is a predictive version of collision avoidance: if the boid does a good job of matching velocity with its neighbors, it is unlikely that it will collide with any of them any time soon. 
    btVector3 v(0,0,0);//or sum
    
    //in a huge flock spread over vast distances, an individual bird must have a localized and filtered perception of the rest of the flock. A bird might be aware of three categories: itself, it's two or three nearest neighbors, and the rest of the flock.
    //those close enough to be considered neighbors of the specified actor
    int neighborCount = 0;
    //The neighborhood is defined as a spherical zone of sensitivity centered at the boid's local origin.
    btScalar neighborhoodSphericalZone = 40.0;// alos known as the neighbor radius
    
    
    //If an agent is found within the radius, its velocity is added to the computation vector, and the neighbor count is incremented.
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            
            if (distance > 0 && distance < neighborhoodSphericalZone){
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
    v = v * actor->m_maxSpeed;
    
    btVector3 steer = v - actor->m_velocity;
    steer = steer.normalize() * actor->m_maxForce;
    
    return steer;
    
}

btVector3 Flock::FlockCentering(const Boid *actor) const{
    
    //boids stay near one another (flock centering or cohesion)
    //Flock Centering: attempt to stay close to nearby flockmates
    //Flock centering makes a boid want to be near the center of the flock.
    //Cohesion is a behavior that causes agents to steer towards the "center of mass" - that is, the average position of the agents within a certain radius.
    
    btVector3 p(0,0,0);//or sum
    int neighborCount = 0;
    btScalar neighborhoodSphericalZone = 40.0;// alos known as the neighbor radius
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        btRigidBody*otherActor = m_boids[b]->m_body;
        
        if (otherActor != actor->m_body)
        {
            btScalar distance = actor->m_body->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition());
            
            if (distance > 0 && distance < neighborhoodSphericalZone){
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


//mult, means mulitply ()
//sub means subtract
//add is addition
//div is divide
//mag is vector length
//limit is the normal or direction of a vector multiplied to a certain amount
void Flock::FlockBoids(){

    for (unsigned int b = 0; b < m_boids.size(); ++b){
        
        //The computational abstraction that combines process, procedure, and state is called an actor
        Boid* actor = m_boids[b];
        btRigidBody* actorBody = actor->m_body;
        
        
        //std::vector<SphereObstacle *>& obstacles = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->obstacles;
        
        btVector3 separation = CollisionAvoidance(actor);//seperation
        btVector3 alignment = VelocityMarching(actor);//alignement
        btVector3 cohesion = FlockCentering(actor);//cohesion
        
        //btVector3 tempVel = alignment + cohesion + separation;
        //tempVel.normalize();
        //actorBody->setLinearVelocity(tempVel);
        //actorBody->applyCentralForce(tempVel);
        
        
        separation = separation * 1.5;
        alignment = alignment * 1.0;
        cohesion = cohesion * 1.0;
        
        m_boids[b]->applyForce(separation);
        m_boids[b]->applyForce(alignment);
        m_boids[b]->applyForce(cohesion);
        
        //An acceleration requests is used to determine which way to steer the boid.
        //The easiest way to combine acceleration requests is to average them. Because of the included "strength" factors, this is actually a weighted average.
        //Prioritized acceleration allocation is based on a strict priority ordering of all component behaviors, hence of the consideration of their acceleration requests.
        //The magnitude of each request is measured and added into another accumulator.
        //This process continues until the sum of the accumulated magnitudes gets larger than the maximum acceleration value, which is a parameter of each boid.
        
        
        
        
        
        /*
        btVector3 bvel = actorBody->getLinearVelocity();
        btVector3 bgravity = actorBody->getGravity() * 0.1;
        btVector3 bdir = btVector3(0, 1, 1);
        btTransform btrans(actor->getOrientation());
        btVector3 up(0, 1, 0);
        btVector3 btop = btrans * up;
        btVector3 front = btrans * btVector3(1, 0, 0);
        btVector3 bdir1 = bvel.safeNormalize();
        btVector3  avel = actor->getAngularVelocity(); 
        btVector3 bthrust = 3.5 * front; //move forward 
        btVector3 bdrag = - 4 * bvel; //resist movement forward
        btVector3 blift = - 2.0 * bgravity * bvel.length(); //pressure agains gravity
        //actorBody->applyCentralForce(bthrust + blift + bgravity + bdrag);
        actorBody->applyCentralForce(bthrust + bgravity);
        actorBody->applyTorque(2 * front.cross(bdir) - 5.0 * avel);
        actorBody->applyTorque(- 0.5 * up);
        actorBody->applyTorque(0.5 * btop.cross(up) - 5 * avel);
        */
    }


    
        //std::vector<SphereObstacle*> collShapes = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->obstacles;
        
        
        
        //    btRigidBody* body0 = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->body000;
        //    btScalar mass = body0->getInvMass();
        //    btVector3 vel = body0->getLinearVelocity();
        //    btVector3 gravity = body0->getGravity();
        //    btVector3 dir = btVector3(0, 0, 1);
        //    btVector3 thrust = 7.0 * dir;
        //    btVector3 drag = -3 * vel;
        //    btVector3 lift = - 0.5 * gravity * vel.length();
        //    body0->applyCentralForce(thrust + lift + gravity + drag );
    
}


void Flock::Update(){

    for (unsigned int b = 0; b < m_boids.size(); ++b){
        
        Boid *actor = m_boids[b];
        
        //update velocity
        actor->m_velocity = actor->m_velocity + actor->m_acceleration;
        
        //Limit velocity speed
        actor->m_velocity = actor->m_velocity.normalize() * actor->m_maxSpeed;
        
        //set position
        actor->m_position = actor->m_body->getCenterOfMassPosition() + actor->m_velocity;
        
        //reset acceleration to 0 each cycle
        actor->m_acceleration = actor->m_acceleration * 0.0;
    }

}


void Flock::Borders(){
    
    
    //wrap around
    //apply force when out of bound of x 
    //apply force when out of bound of z 
    //apply gravity force force when too high 
    //apply lift force when too low  
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        Boid *actor = m_boids[b];
        actor->m_position = actor->m_body->getCenterOfMassPosition();
        
        btScalar x = actor->m_position.x();
        btScalar y = actor->m_position.y();
        btScalar z = actor->m_position.z();
        
        //x directions
        if(x < (-(m_borderWidth) + (actor->m_radius) ) ) { //going to far in negative x direction
            //x = m_borderWidth +(actor->m_radius); 
        }
        if(x > ( (m_borderWidth)-(actor->m_radius) ) ) {//going to far in positive x direction
            //x = -(actor->m_radius); 
        }
        
        //z directions
        if(z < (-(m_borderWidth) + (actor->m_radius) ) ) {//going to far in negative z direction
            //z = (actor->m_width)+(actor->m_radius); 
        }
        if(z > ( (m_borderWidth)-(actor->m_radius) ) ) {//going to far in positive z direction
            //z = -(actor->m_radius); 
        }
        
        //y direction
        if(y < (actor->m_height) ) { //going to far doing or falling too much
            //y = -(actor->m_radius); 
        }
        if(y > m_borderHeight-(actor->m_height) ) {//going to far up or rising too high
            //y = (actor->m_width)+(actor->m_radius); 
        }
        
        actor->m_position = btVector3(x, y, z);
    }
}


void Flock::Run(){
    
    FlockBoids();
    Update();
    //Borders();
    
    for (unsigned int b = 0; b < m_boids.size(); ++b){
        m_boids[b]->m_body->applyCentralForce(m_boids[b]->m_velocity);
        //m_boids[b]->m_body->setLinearVelocity(m_boids[b]->m_velocity);
        //m_boids[b]->m_transform.setOrigin(m_boids[b]->m_position);
    }
    
}


