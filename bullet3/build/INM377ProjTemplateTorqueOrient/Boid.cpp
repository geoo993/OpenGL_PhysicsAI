#include "Boid.h"

//boid default constructor
Boid::Boid(){
    
    m_body = nullptr; 
    m_hullShape = nullptr;
    
    m_transform = btTransform();
    m_hullShape = new btConvexHullShape();
}
//boid constructor with rigidbody parameter
Boid::Boid(btRigidBody* body){
    
    m_body = body; 
    m_hullShape = nullptr;
    
    m_transform = btTransform();
    m_hullShape = new btConvexHullShape();
}

//boid destructor
Boid::~Boid(){
    delete m_body;
    delete m_hullShape;
}

//Set boid position
void  Boid::SetPosition( const btVector3 &position){
    
    //creating boids shape
    int r = rand() % 3;    
    if ( r == 1){
        m_hullShape->addPoint(btVector3(3.5, 0, 0));
        m_hullShape->addPoint(btVector3(0, 1.0, 0));
        m_hullShape->addPoint(btVector3(0, 0, 1.8));
        m_hullShape->addPoint( btVector3(0, 0, -1.8));
    }else if (r == 2){
        m_hullShape->addPoint(btVector3(3.0, 0, 0));
        m_hullShape->addPoint(btVector3(0, 1.0, 0));
        m_hullShape->addPoint(btVector3(1.5, 0, 1.0));
        m_hullShape->addPoint(btVector3(1.5, 0, -1.0));
        m_hullShape->addPoint(btVector3(-0.6, 0, 1));
        m_hullShape->addPoint(btVector3(-0.6, 0, -1));
        m_hullShape->addPoint(btVector3(-1.5, 0, 0.5));
        m_hullShape->addPoint(btVector3(-1.5, 0, -0.5));
    }else{
        m_hullShape->addPoint(btVector3(3.0, 0, 0));
        m_hullShape->addPoint(btVector3(0, 1.0, 0));
        
        m_hullShape->addPoint(btVector3(2.8, 0, 0.2));
        m_hullShape->addPoint(btVector3(2.5, 0, 0.4));
        m_hullShape->addPoint(btVector3(2.2, 0, 0.8));
        m_hullShape->addPoint(btVector3(1.9, 0, 1.1));
        m_hullShape->addPoint(btVector3(1.6, 0, 1.3));
        m_hullShape->addPoint(btVector3(1.3, 0, 1.5));
        m_hullShape->addPoint(btVector3(1.0, 0, 1.8));
        m_hullShape->addPoint(btVector3(0.7, 0, 2.0));
        m_hullShape->addPoint(btVector3(0.3, 0, 1.8));
        m_hullShape->addPoint(btVector3(0.0, 0, 1.5));
        m_hullShape->addPoint(btVector3(-0.3, 0, 1.0));
        
        m_hullShape->addPoint(btVector3(2.8, 0, -0.2));
        m_hullShape->addPoint(btVector3(2.5, 0, -0.4));
        m_hullShape->addPoint(btVector3(2.2, 0, -0.8));
        m_hullShape->addPoint(btVector3(1.9, 0, -1.1));
        m_hullShape->addPoint(btVector3(1.6, 0, -1.3));
        m_hullShape->addPoint(btVector3(1.3, 0, -1.5));
        m_hullShape->addPoint(btVector3(1.0, 0, -1.8));
        m_hullShape->addPoint(btVector3(0.7, 0, -2.0));
        m_hullShape->addPoint(btVector3(0.3, 0, -1.8));
        m_hullShape->addPoint(btVector3(0.0, 0, -1.5));
        m_hullShape->addPoint(btVector3(-0.3, 0, -1.0));
        
    }
    
    //setting position
    m_transform.setIdentity();
    m_transform.setOrigin(position);
    
    //setting mass
    btVector3 bLocalInertia(0,0,0);
    m_hullShape->calculateLocalInertia(bGet(BoidsValues::BMASS), bLocalInertia);
    
}

//activate boid
void Boid::Activate(){
    
    m_body->setAnisotropicFriction(m_hullShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    m_body->setFriction(0.5);
    m_body->setLinearVelocity(btVector3(1,0,0));
    m_body->activate(true);
}

//returns a force that will be used to move a boid toward a desired position, this is used in the cohesion
btVector3 Boid::Seek(const btVector3 &target) const{
    
    btVector3 desired = target - m_body->getCenterOfMassPosition();
    desired.normalize();
    desired = desired * bGet(BoidsValues::BMAXSPEED); //m_maxSpeed;
    
    btVector3 steer = desired - m_body->getLinearVelocity();
    steer = steer.normalize() * bGet(BoidsValues::BMAXFORCE);
    
    return steer;
}

//returns a lift force used to make boids rise off the ground
btVector3 Boid::LiftForce(const btScalar & boundary) const{
    
    //lift
    if( (boundary - m_body->getCenterOfMassPosition().y())  < 20.0){
        return btVector3(0, -(bGet(Boid::BoidsValues::BLift)), 0);
    }else if( (m_body->getCenterOfMassPosition().y()) < 20.0 ) { 
        return btVector3(0,bGet(Boid::BoidsValues::BLift), 0);
    }else{
        return btVector3(0,0, 0);
    }
    
}

//returns torque force that will make boids turn away from obstacles 
btVector3 Boid::AvoidanceForce(const std::vector<Obstacle *>& obstacles) const{
    
    btVector3 bposition = btVector3(m_body->getCenterOfMassPosition().x(), 0.0,m_body->getCenterOfMassPosition().z());
    btVector3 oAvoidance = btVector3(0,0,0);
    
    for (unsigned int o = 0; o < obstacles.size(); ++o){
        Obstacle * obstacle = obstacles[o];
        if (obstacle->InPath(bposition))
        {
            
            btVector3 actorDirectionToObstacle = m_body->getCenterOfMassPosition() - obstacles[0]->getCentre();
            actorDirectionToObstacle.safeNormalize();
            btScalar angleBasedOnDirection = Extension::getAngleBetweenTwoPoints(
                                                                                 actorDirectionToObstacle.x(),
                                                                                 0,
                                                                                 actorDirectionToObstacle.z(),
                                                                                 0.0,0.0,0.0);
            
            btScalar actorLocalAngle = Extension::getAngleBetweenTwoPoints(GetHeading().x()
                                                                           ,0,GetHeading().z(),
                                                                           0.0,0.0,0.0);

            oAvoidance = obstacle->GetAvoidanceForce(angleBasedOnDirection, actorLocalAngle, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)));
            
//            if(angleBasedOnDirection > 0.0 && angleBasedOnDirection < 90.0 ){
//                
//                if( actorLocalAngle > 90.0 && actorLocalAngle < 180.0 ){
//                    //45->-125     then left is 45 to 125 and right is 125 to -125
//                    bool right = ( (actorLocalAngle > 125.0 && actorLocalAngle < 180.0) || (actorLocalAngle <= -125.0 && actorLocalAngle > -180.0) ) ;
//                    bool left = (actorLocalAngle > 45.0 && actorLocalAngle <= 125.0);
//                    if (right){
//                        oAvoidance = btVector3( 0, -(bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//right
//                    }
//                    if (left){
//                        oAvoidance = btVector3( 0, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//left
//                    }
//                }
//            }else if(angleBasedOnDirection >= 90.0 && angleBasedOnDirection < 180.0 ){
//                if( actorLocalAngle > 0.0 && actorLocalAngle < 90.0 ){
//                    //-45->125     then left is -45 to 45 and right is 45 to 125
//                    bool right = (actorLocalAngle < 125.0 && actorLocalAngle > 45.0);
//                    bool left = ((actorLocalAngle <= 45.0 && actorLocalAngle > 0.0) || (actorLocalAngle < 0.0 && actorLocalAngle > -45.0));
//                    if (right){
//                        oAvoidance = btVector3( 0, -(bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//right
//                    }
//                    if (left){
//                        oAvoidance = btVector3( 0, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//left
//                    }
//                }
//                
//            }else if(angleBasedOnDirection < 0.0 && angleBasedOnDirection > -90.0){
//                if( actorLocalAngle < -90.0 && actorLocalAngle > -180.0 ){
//                    
//                    //-45->125     then left is -125 to 125 and right is -45 to -125
//                    bool right = (actorLocalAngle < -45.0 && actorLocalAngle > -125.0);
//                    bool left = ((actorLocalAngle <= -125.0 && actorLocalAngle > -180.0) || (actorLocalAngle < 180.0 && actorLocalAngle > 125.0));
//                    
//                    if (right){
//                        oAvoidance = btVector3( 0, -(bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//right
//                    }
//                    if (left){
//                        oAvoidance = btVector3( 0, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//left
//                    }
//                }
//            }else if(angleBasedOnDirection <= -90.0 && angleBasedOnDirection > -180.0 ){
//                if( actorLocalAngle < 0.0 && actorLocalAngle > -90.0){
//                    
//                    //45->-125     then left is -45 to -125 and right is -45 to 45
//                    bool right = ((actorLocalAngle < 0.0 && actorLocalAngle > -45.0) || (actorLocalAngle < 45.0 && actorLocalAngle > 0.0));
//                    bool left = (actorLocalAngle <= -45.0 && actorLocalAngle > -125.0);
//                    
//                    if (right){
//                        oAvoidance = btVector3( 0, -(bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//right
//                    }
//                    if (left){
//                        oAvoidance = btVector3( 0, (bGet(Boid::BoidsValues::BMAXAVOIDANCEFORCE)),0 );//left
//                    }
//                    
//                }
//            }
        
            
        } 
        
    }
    
    return oAvoidance;
    
}

//This is used to apply steering torque force to each boid in the flock to turn them back in the scene
void Boid::SteerBack(){
    
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
    btScalar angleFromWorld = Extension::getAngleBetweenTwoPoints(m_body->getCenterOfMassPosition().x()
                                                                  ,0,m_body->getCenterOfMassPosition().z(),
                                                                  0.0,0.0,0.0);
    btScalar angleFromLocal = Extension::getAngleBetweenTwoPoints(GetHeading().x()
                                                                  ,0,GetHeading().z(),
                                                                  0.0,0.0,0.0);
    if (m_body->getCenterOfMassPosition().length() > bGet(Boid::BoidsValues::BBORDERBOUNDARY)){
        
        btVector3 steer(0, bGet(Boid::BoidsValues::BROTATEBACK), 0);
        
        if(angleFromWorld > 0.0 && angleFromWorld < 90.0 ){
            if( (angleFromLocal > 0.0 && angleFromLocal < 45.0) || (angleFromLocal <= 0.0 && angleFromLocal > -125.0) ){
                m_body->applyTorque(steer);
            }
        }else if(angleFromWorld >= 90.0 && angleFromWorld < 180.0 ){
            if( (angleFromLocal > 125.0 && angleFromLocal < 180.0) || (angleFromLocal <= -45.0 && angleFromLocal > -180.0) ){
                m_body->applyTorque(steer);
            }
        }else if(angleFromWorld < 0.0 && angleFromWorld > -90.0){
            if( (angleFromLocal > 0.0 && angleFromLocal < 125.0) || (angleFromLocal <= 0.0 && angleFromLocal > -45.0) ){
                m_body->applyTorque(steer);
            }
        }else if(angleFromWorld <= -90.0 && angleFromWorld > -180.0 ){
            if( (angleFromLocal > 45.0 && angleFromLocal < 180.0) || (angleFromLocal <= -125.0 && angleFromLocal > -180.0) ){
                m_body->applyTorque(steer);
            }
        }
    }
    
}



