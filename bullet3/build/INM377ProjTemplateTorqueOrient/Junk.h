//
//  Junk.h
//  BULLET_PHYSICS
//
//  Created by GEORGE QUENTIN on 02/04/2017.


/*

 
 static int count = 0;
 static btVector3 btorqueTurnLeft;
 static btVector3 btorqueTurnRight;
 static btVector3 btorqueSpinLeft;
 static btVector3 btorqueSpinRight;
 
 static btScalar adjustLeftTorque = 5.0;
 static btScalar adjustRightTorque = 5.0;
 
 static void functionsTest(btDynamicsWorld *world){
 //steer(world, timeStep);
 
 
 
 // std::vector<Boid*> boids = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boidObjects;
 //   
 //static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.Run();
 
 btRigidBody* bbody0 = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boid;
 
 //    count += 1.0;
 //    if (count > 10){
 //        count = 0;
 //    }
 
 
 //// *******  use these to add torque force *******/////////

/*
btQuaternion orientation = bbody0->getOrientation();//orientation in 
btMatrix3x3 bMatOrientation = btMatrix3x3(orientation); // quat to matrix
bMatOrientation.setEulerYPR(orientation.getX() , orientation.getY() + count, orientation.getZ());

btVector3 baxisAngles = orientation.getAxis();
btScalar getAngle = orientation.getAngle();
btScalar bangle = btAcos(( bMatOrientation[0][0] + bMatOrientation[1][1] + bMatOrientation[2][2] - 1)/2);
btScalar bdegreeangle = (bangle * 180.0 ) / M_PI ; //btDegrees(bangle);
btTransform btrans(orientation);
btVector3 bfront = btrans * btVector3(1, 0, 0);//forward vector of the boid
btVector3 bforward = btVector3(bMatOrientation[0][0], bMatOrientation[0][1], bMatOrientation[0][2]);
btVector3 bback = bforward.normalize() * -1.0f;
btVector3 bright = btVector3(bMatOrientation[2][0], bMatOrientation[2][1], bMatOrientation[2][2]);
//btVector3 bright = bfront.cross(bdir);//get normal or left/right vector
btVector3 bup = btVector3(bMatOrientation[1][0], bMatOrientation[1][1], bMatOrientation[1][2]);
btVector3 up(0, 1, 0);
btVector3 btop = btrans * up;
btVector3 bbot = btrans * -up;
btVector3 bdown = bup.normalize() * -1.0f;
btVector3 bleft = bright.normalize() * -1.0f;

//btVector3 btorqueOverTime = 2 * bfront.cross(bdir) - 5.0 * avel;
btScalar distanceFromCenterPoint = 10.0;// the greater the value the more spin it will have, meaning the further away you apply angular speed from the center of mass the more it will spin
btVector3 momentArmLeft = (distanceFromCenterPoint * bup);//The distance from the pivot point to the point where the force acts is called the moment arm, it is a vector
btVector3 momentArmRight = (distanceFromCenterPoint * bdown);
btVector3 momentArmLeftSpin = (distanceFromCenterPoint * bleft);
btVector3 momentArmRightSpin = (distanceFromCenterPoint * bright);
btVector3 avel = bbody0->getAngularVelocity();//angular velocity, the spin about an axis through the centre of mass
btVector3 forceOfSpin = -5.0 * (avel);
btorqueTurnLeft = (momentArmLeft + forceOfSpin);//(momentArmRight + forceOfSpin) * bmaxavoidanceforce// 5 is the scalar difference of the force we should apply
//torque is the measurement of how much a force acting on an object causes that object to rotate
btorqueTurnRight = (momentArmRight + forceOfSpin) ;

btorqueSpinLeft = (momentArmLeftSpin + forceOfSpin) ;
btorqueSpinRight = (momentArmRightSpin + forceOfSpin) ;

/// btMatrix3x3 ypr;
//ypr.setEulerYPR(0.2,0.1,0.1);
//bMatOrientation *= ypr //Multiply it by rotation matrix, yaw, pitch, roll
//m_pRigidBody->getWorldTransform().setBasis(bMatOrientation); //set new rotation for the object


*/

////*******  use these to add central force   *******/////////
/*
btScalar bmass = bbody0->getInvMass();
btVector3 bvel = bbody0->getLinearVelocity();
btVector3 bgravity = bbody0->getGravity();
btVector3 bposition = bbody0->getCenterOfMassPosition();
btVector3 bdir = bvel.safeNormalize();//the velocity vector describes the direction of the character. The direction of the velocity vector will control where the character is heading to while its length (or magnitude) will control how much it will move every frame. The greater the length, the faster the character moves.
btScalar maxSeeAhead = 20.0;//this ahead vector length defines how far the character will "see". The greater this is, the earlier the character will start acting to dodge an obstacle, because it will perceive it as a threat even if it's far away.
btVector3 bahead = bposition + (bforward * maxSeeAhead);
btVector3 bahead2 = bposition + (bforward * maxSeeAhead) * 0.5;
//btVector3 bahead = bposition + (bdir * maxSeeAhead);
//btVector3 bahead2 = bposition + (bdir * maxSeeAhead) * 0.5;



//intersecting check 
//We want to perform a collision or intersecting check to test whether either of those two ahead vectors are inside our the obstacle or over our boundary. That's easily accomplished by comparing the distance between the vector's end and the obstacle center or the boundary line.
btScalar maxHeight = 20.0;
btScalar ground = 4.0;
btScalar x = bposition.x();
btScalar y = bposition.y();
btScalar z = bposition.z();
btVector3 intersectPoint = btVector3(0,0,0);
btScalar distanceFromIntersector = 0.0;
btScalar distanceUntilIntersectorOccur = 0.0;
btScalar xIntersect = 0;
btScalar zIntersect = 0;

bool isAhead1OutOfBounds = false;
bool isAhead2OutOfBounds = false;
bool isBothAheadOutOfBounds = false;

bool mostThreathing = false;//If more than one obstacle is blocking the way, then the closest one (the "most threatening") is selected for calculation

btScalar verticalDifference = maxHeight - y;
btScalar verticalPercentageDifference = Extension::percentageWith(verticalDifference, ground, maxHeight);
btScalar pressure = verticalPercentageDifference / 100.0;    // -1 to  1
btVector3 blift = - (1.0 + pressure) * bgravity * bvel.length();
btVector3 bthrust = 2.0  * bforward ;//bfront
btVector3 bdrag = -1.0 * bforward;//bvel;


//going to far in negative x direction
//going to far in positive x direction
//going to far in negative z direction
//going to far in positive z direction
if( (bahead.x() < -50.0) || (bahead.x() > 50.0) || (bahead.z() < -50.0) || (bahead.z() > 50.0) ) { 
    //if (bahead.x() > 20.0){
    //balance = 2.0;
    //reduce velocity
    //turn
    isAhead1OutOfBounds = true;
}else {
    //balance = 1.0;
    isAhead1OutOfBounds = false;
}

if( (bahead2.x() < -50.0) || (bahead2.x() > 50.0) || (bahead2.z() < -50.0) || (bahead2.z() > 50.0) ) { 
    //if (bahead2.x() > 20.0){  
    //balance = 2.0;
    //reduce velocity
    //turn
    isAhead2OutOfBounds = true;
}else {
    //balance = 1.0;
    isAhead2OutOfBounds = false;
}

if (isAhead1OutOfBounds){ //&& isAhead2OutOfBounds){
    //isBothAheadOutOfBounds = true;
    
    if ( (bahead.x() > 50.0) ){
        xIntersect = bahead.x() - 50.0;
    }
    if( (bahead.x() < -50.0) ) { 
        xIntersect = bahead.x() + 50.0;
    }
    
    if ( (bahead.z() > 50.0) ){
        zIntersect = bahead.z() - 50.0;
    }
    if( (bahead.z() < -50.0) ) { 
        zIntersect = bahead.z() + 50.0;
    }
    
    intersectPoint = bahead - btVector3(xIntersect,bahead.y(),zIntersect);
    
    distanceFromIntersector = bahead.distance( intersectPoint);
    distanceUntilIntersectorOccur = intersectPoint.distance(bposition);
    
    //bbody0->setLinearVelocity(btVector3(0,0,0));
    //bbody0->applyTorque(btorqueSpinLeft);//allign to the left
    //bbody0->applyCentralForce(bthrust + blift + bgravity + bdrag);
    
    bbody0->applyCentralForce(bdrag * distanceUntilIntersectorOccur);// + blift + bgravity);
    
    
}else{
    
    //bbody0->setLinearFactor(btVector3(1,0,1));
    //bbody0->setAngularFactor(btVector3(1,0,0));
    bbody0->applyCentralForce(bthrust);// + blift + bgravity); //+ bdrag);
    
    
}

//bbody0->getWorldTransform().setBasis(bMatOrientation); //set new rotation for the object
//btVector3 m = bbody0->getWorldTransform().getBasis()*btVector3(strafe,move,rise);


bbody0->applyTorque(btorqueTurnLeft);//allign to the left
bbody0->applyTorque(btorqueTurnRight);//allign to the right
//bbody0->applyTorque(btorqueSpinLeft );//allign to the left
//bbody0->applyTorque(btorqueSpinRight);//allign to the left



//bbody0->applyTorque(btorqueTurnLeft);//allign to the left
//bbody0->applyTorque(- 0.5 * up);
//bbody0->applyTorque(btorqueTurnRight);//allign to the right

//bbody0->applyTorque(2 * front.cross(bdir) - 5.0*avel);
//bbody0->applyTorque(- 0.5 * up);
//bbody0->applyTorque(0.5 * btop.cross(up) - 5*avel);


std::cout << std::endl;

std::cout << "is seeing ahead1: " << isAhead1OutOfBounds << std::endl;
std::cout << "is seeing ahead2: " << isAhead2OutOfBounds << std::endl;
std::cout << "is seeing both ahead: " << isBothAheadOutOfBounds << std::endl;
std::cout << "intersertion point, x: " << intersectPoint.x() << " y: " << intersectPoint.y() << " z: " << intersectPoint.z() << std::endl;
std::cout << "yaw: " << orientation.getX() << std::endl;
std::cout << "distance From intersertion point: " << distanceFromIntersector << std::endl;
std::cout << "distance until intersertion occur: " << distanceUntilIntersectorOccur << std::endl;
std::cout << "top, x: " << btop.x() << " y: " << btop.y() << " z: " << btop.z() << std::endl;
std::cout << "bottom, x: " << bbot.x() << " y: " << bbot.y() << " z: " << bbot.z() << std::endl;
std::cout << "right, x: " << bright.x() << " y: " << bright.y() << " z: " << bright.z() << std::endl;
std::cout << "left, x: " << bleft.x() << " y: " << bleft.y() << " z: " << bleft.z() << std::endl;
std::cout << "forward, x: " << bforward.x() << " y: " << bforward.y() << " z: " << bforward.z() << std::endl;
std::cout << "front, x: " << bfront.x() << " y: " << bfront.y() << " z: " << bfront.z() << std::endl;
std::cout << "ahead, x: " << bahead.x() << " y: " << bahead.y() << " z: " << bahead.z() << std::endl;
std::cout << "ahead2, x: " << bahead2.x() << " y: " << bahead2.y() << " z: " << bahead2.z() << std::endl;
std::cout << "direction, x: " << bdir.x() << " y: " << bdir.y() << " z: " << bdir.z() << std::endl;

std::cout << "b left, x: " << btorqueTurnLeft.x() << " y: " << btorqueTurnLeft.y() << " z: " << btorqueTurnLeft.z() << std::endl;
std::cout << "b right, x: " << btorqueTurnRight.x() << " y: " << btorqueTurnRight.y() << " z: " << btorqueTurnRight.z() << std::endl;
std::cout << "velocity, x: " << bvel.x() << " y: " << bvel.y() << " z: " << bvel.z() << std::endl;
std::cout << "angular velocity, x: " << avel.x() << " y: " << avel.y() << " z: " << avel.z() << std::endl;
std::cout << "angle: " <<  bangle << std::endl;
std::cout << "degree angle: " <<  bdegreeangle << std::endl;
std::cout << "angles, x: " << baxisAngles.x() << " y: " << baxisAngles.y() << " z: " << baxisAngles.z() << std::endl;

std::cout << "position, x: " << x << " y: " << y << " z: " << z << std::endl;

std::cout << "bthrust, x: " << bthrust.x() << " y: " << bthrust.y() << " z: " << bthrust.z() << std::endl;

std::cout << "lift, x: " << blift.x() << " y: " << blift.y() << " z: " << blift.z() << std::endl;
std::cout << "gravity, x: " << bgravity.x() << " y: " << bgravity.y() << " z: " << bgravity.z() << std::endl;
std::cout << "y: " << y << ", vertical difference: " << verticalDifference << ", vertical percentage difference: " << verticalPercentageDifference << ", pressure: " << pressure << std::endl; 



}




 
 
 
 //btransform.setOrigin(bposition);
 //bbody->applyCentralForce(bvelocity);
 //bbody->setLinearVelocity(bvelocity);
 //bbody->applyTorque(btorqueTurnLeft);//allign to the left
 //bbody->applyTorque(btorqueTurnRight);//allign to the right
 
 
 
 //static void applyForce(btVector3 force) {
 //    bacceleration = bacceleration + (force);
 //}
 
 
 bvelocity = bvelocity + bacceleration;
 bvelocity = bvelocity.normalize() * maxspeed;
 bposition =  bposition + bvelocity;
 bacceleration = bacceleration * 0.0;

 
 
 
 
 //       btVector3 target = btVector3(0,0,0);
 //       bdesired = target - bbody->getCenterOfMassPosition();
 //       bdesired = bdesired.normalize() * bmaxspeed;
 //        
 //       bsteer = bdesired - bvelocity;
 //       bsteer = bsteer.normalize() * bmaxforce;
 //       bbody->applyCentralForce(bsteer);
 
 //bbody->applyForce(bvelocity, bsteer);
 //applyForce(bsteer);
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 if( (bahead.x() < -bboundary2) || (bahead.x() > bboundary2) || (bahead.z() < -bboundary2) || (bahead.z() > bboundary2) ) { 
 isAhead1OutOfBounds = true;
 }else {
 isAhead1OutOfBounds = false;
 }
 
 if (isAhead1OutOfBounds){ 
 
 if ( (bahead.x() > 50.0) ){
 xIntersect = bahead.x() - 50.0;
 }
 if( (bahead.x() < -50.0) ) { 
 xIntersect = bahead.x() + 50.0;
 }
 
 if ( (bahead.z() > 50.0) ){
 zIntersect = bahead.z() - 50.0;
 }
 if( (bahead.z() < -50.0) ) { 
 zIntersect = bahead.z() + 50.0;
 }
 
 intersectPoint = bahead - btVector3(xIntersect,bahead.y(),zIntersect);
 
 distanceFromIntersector = bahead.distance( intersectPoint);
 distanceUntilIntersectorOccur = intersectPoint.distance(bposition);
 
 //bbody0->setLinearVelocity(btVector3(0,0,0));
 //bbody->applyTorque(btorqueSpinLeft);//allign to the left
 //bbody0->applyCentralForce(bthrust + blift + bgravity + bdrag);
 
 //bbody->applyCentralForce(bdrag * distanceUntilIntersectorOccur);// + blift + bgravity);
 
 //       btTransform btrans = bbody->getWorldTransform();
 //       btVector3 bdir = bvelocity.safeNormalize();
 //       btVector3 bfront = btrans * btVector3(1, 0, 0);
 //       btVector3 target = bfront.cross(bdir);
 //       bdesired = target - bposition;
 //       bdesired = bdesired.normalize() * bmaxspeed;
 //        
 //       bsteer = bdesired - bvelocity;
 //       bsteer = bsteer.normalize() * bmaxforce;
 //        
 //        
 //bbody->applyForce(bvelocity, bsteer);
 //bbody->applyCentralForce(bsteer);
 //applyForce(bsteer);
 
 
 
 
 //if (angletodegrees > 100){
 //bbody->applyTorque(btVector3(0, distanceUntilIntersectorOccur ,0));
 //bbody->applyTorque(btVector3(0, -(distanceUntilIntersectorOccur ) ,0));
 //btVector3 TorqueForce(0, rotateBackForce, 0);
 //bbody->applyTorque(TorqueForce);
 //}
 //bbody->setDamping(1.0, 0.5);
 //btVector3 torque(0,distanceUntilIntersectorOccur,0);
 //bbody->getInvInertiaTensorWorld().inverse()*(bbody->getWorldTransform().getBasis() * torque);
 }
 //bbody->setLinearVelocity(bthrust);
 //bbody->applyCentralForce(bthrust);
 
 
 
 
 
 
 std::cout << std::endl;
 //std::cout << "is seeing ahead1: " << isAhead1OutOfBounds << std::endl;
 std::cout << "length: " << bbody->getCenterOfMassPosition().length() << std::endl;
 std::cout << "from local angle: " << ( angleFromLocal ) << ", from world angle: " << ( angleFromWorld ) << ", bangle: " << (bangle * 180 / M_PI) << ", bangle2: " << (bangle2 * 180 / M_PI) << ", angle: " << angle << ", angle in degree: " << angletodegrees << std::endl;
 
 //std::cout << "velocity, x: " << bvelocity.x() << " y: " << bvelocity.y() << " z: " << bvelocity.z() << std::endl;
 //std::cout << "angular velocity, x: " << bavelocity.x() << " y: " << bavelocity.y() << " z: " << bavelocity.z() << std::endl;
 std::cout << "position, x: " << bposition.x() << " y: " << bposition.y() << " z: " << bposition.z() << std::endl;
 
 
 //    btQuaternion orient = bbody->getOrientation();//orientation in 
 //    btMatrix3x3 bMatOrient= btMatrix3x3(orient); // quat to matrix
 //    bMatOrient.setEulerYPR(orient.getX() , orient.getY() + 20 , orient.getZ());
 //    bbody->getWorldTransform().setBasis(bMatOrient); //set new rotation for the object
 //    
 ////    btTransform tr;
 ////    tr.setIdentity();
 ////    btQuaternion quat = bbody->getOrientation();
 ////    quat.setEuler(quat.x(),quat.z(),quat.y() ); //or quat.setEulerZYX depending on the ordering you want
 ////    tr.setRotation(quat);
 //    //bbody->setCenterOfMassTransform(tr);
 //    
 //    
 //    btScalar angle = bforward.angle(btVector3(bbody->getCenterOfMassPosition().x(), 0.0, bbody->getCenterOfMassPosition().z()));
 //    btScalar angletodegrees = (angle * 180 / M_PI);
 //    
 //    
 //    std::cout << std::endl;
 //    std::cout << "angle " << angletodegrees << std::endl;
 //
 
 
 
 //    if( bdistanceZp < bboundaryintersectiondistance){
 //        //bimpluseZ = bdistanceZp * 0.2;
 //        //bbody->applyCentralImpulse(btVector3(0,0,bimpluseZ));
 //        //bbody->applyCentralForce(btVector3(0, 0, -bsidesforce));
 //        
 //        btScalar pScale = Extension::percentageWith(bdistanceZp, 0, bboundaryintersectiondistance); 
 //        btClamp(pScale, btScalar(0.0), btScalar(100.0));
 //        pScale = 100.0 - pScale;
 //        btScalar temp = bsidesforce * (pScale/100.0);
 //        //bbody->applyCentralForce(btVector3(0,0,-temp));
 //        //bbody->applyCentralImpulse(btVector3(0,0,-temp));
 //        
 //    }else if( bdistanceZn < bboundaryintersectiondistance ) { 
 //        //bimpluseZ = bdistanceZn * 0.2;
 //        //bbody->applyCentralImpulse(btVector3(0,0,bimpluseZ));
 //        //bbody->applyTorque(btVector3(0,0,25));
 //        //bbody->applyCentralForce(btVector3(0,0,bsidesforce));
 //        
 //        btScalar pScale = Extension::percentageWith(bdistanceZn, 0, bboundaryintersectiondistance); 
 //        btClamp(pScale, btScalar(0.0), btScalar(100.0));
 //        pScale = 100.0 - pScale;
 //        btScalar temp = bsidesforce * (pScale/100.0);
 //        bbody->applyCentralForce(btVector3(0,0,temp));
 //        //bbody->applyCentralImpulse(btVector3(0,0,temp));
 //    }
 //    //bbody->setAngularVelocity(btVector3(0,0,0));
 
 
 
 
 //    if( bdistanceXp < bboundaryintersectiondistance){
 //        
 //        btScalar pScale = Extension::percentageWith(bdistanceXp, 0, bboundaryintersectiondistance); 
 //        btClamp(pScale, btScalar(0.0), btScalar(100.0));
 //        pScale = 100.0 - pScale;
 //        btScalar temp = bsidesforce * (pScale/100.0);
 //        std::cout << std::endl;
 //        std::cout << "scale: " << pScale << ", temp: " << temp << std::endl;
 //        //bbody->applyCentralImpulse(btVector3(-temp, 0, 0));
 //        //bbody->applyCentralForce(btVector3(-temp, 0, 0));
 //        //bbody->applyCentralForce(btVector3(-bsidesforce, 0, 0));
 //        //bbody->applyCentralImpulse(btVector3(-temp,0,0));
 //    }else if (bdistanceXn < bboundaryintersectiondistance) { 
 //        btScalar pScale = Extension::percentageWith(bdistanceXn, 0, bboundaryintersectiondistance); 
 //        btClamp(pScale, btScalar(0.0), btScalar(100.0));
 //        pScale = 100.0 - pScale;
 //        btScalar temp = bsidesforce * (pScale/100.0);
 //        //bbody->applyCentralForce(btVector3(temp ,0,0));
 //        //bbody->applyCentralForce(btVector3(bsidesforce, 0, 0));
 //        //bbody->applyCentralImpulse(btVector3(temp,0,0));
 //    }
 
 
 
 bdistanceXp = bboundary - bposition.x();
 bdistanceXn = bposition.x();
 bdistanceYp = bboundary - bposition.y();
 bdistanceYn = bposition.y();
 bdistanceZp = bboundary - bposition.z();
 bdistanceZn = bposition.z();
 
 
 //    btScalar distanceFromCenterPoint = 6.0;
 //    btVector3 momentArmLeft = (distanceFromCenterPoint * bup);
 //    btVector3 momentArmRight = (distanceFromCenterPoint * bdown);
 //    btVector3 forceOfSpin = -4.0 * (bavelocity);
 //    //btorqueTurnLeft = (momentArmLeft + forceOfSpin);
 //    //btorqueTurnRight = (momentArmRight + forceOfSpin) ;
 
 
 
 btVector3 intersectPoint = btVector3(0,0,0);
 btScalar distanceFromIntersector = 0.0;
 btScalar distanceUntilIntersectorOccur = 0.0;
 btScalar xIntersect = 0;
 btScalar zIntersect = 0;
 
 
 btVector3 baxisAngles = orientation.getAxis();
 btScalar bangle = orientation.getAngle();
 //orientation.setRotation(btVector3( 0.0, 0.0, 1.0 ), angle);
 //orientation.setRotation(btVector3( 0.0, 1.0, 0.0 ), angleZ);
 //btAtan2(bbody->getCenterOfMassPosition().z(), bbody->getCenterOfMassPosition().x());
 btScalar angle = bbody->getCenterOfMassPosition().angle(btVector3(bbody->getCenterOfMassPosition().x(), 0.0, bbody->getCenterOfMassPosition().z()));
 btScalar angletodegrees = (angle * 180 / M_PI);
 
 
 */
