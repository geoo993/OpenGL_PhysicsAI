/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT 1.f

#include "INM377ProjTemplateForcesTorqueOrient.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"

#if 0
extern btAlignedObjectArray<btVector3> debugContacts;
extern btAlignedObjectArray<btVector3> debugNormals;
#endif 

static GLDebugDrawer	sDebugDrawer;


INM377ProjTemplateTorqueOrient::INM377ProjTemplateTorqueOrient()
{
    
    initialiseFlock();
    
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.0));
}


void INM377ProjTemplateTorqueOrient::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	//float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1./60.0,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

//	displayText();
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();

	swapBuffers();

}


void INM377ProjTemplateTorqueOrient::displayText()
{
	int lineWidth=440;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		
		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press 'p' to change CCD mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press '.' or right mouse to shoot bullets");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"space to restart, h(elp), t(ext), w(ire)");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}	

}



void INM377ProjTemplateTorqueOrient::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	displayText();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();
	swapBuffers();
}

void INM377ProjTemplateTorqueOrient::createGround(){
    
    ///create a few basic rigid bodies
    btBoxShape* box = new btBoxShape(btVector3(btScalar(110.),btScalar(1.),btScalar(110.)));
    //	box->initializePolyhedralFeatures();
    btCollisionShape* groundShape = box;
    
    //	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
    
    m_collisionShapes.push_back(groundShape);
    //m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
    m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,0));
    
    //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
    {
        btScalar mass(0.0);
        
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.0f);
        
        btVector3 localInertia(0,0,0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass,localInertia);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(0.5f);
        
        //body->setRollingFriction(0.3);
        //add the body to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
}

void INM377ProjTemplateTorqueOrient::initialiseFlock(){
    
    std::vector<Boid*>	boidObjects;
    std::vector<Obstacle *> obstacles;
    
    boidObjects.reserve(NUMBER_OF_BOIDS);
    obstacles.reserve(NUMBER_OF_OBSTACLES);
    collisionBodies.reserve(NUMBER_OF_OBSTACLES);
    
    for (unsigned int a = 0; a < NUMBER_OF_BOIDS; ++a){
        boidObjects.push_back(new Boid);
    }
    
    std::vector<btVector3> obstaclesPositions = {
        btVector3(-5, 1, 0),
        btVector3(-30, 1, 20),
        btVector3(0, 1, 35),
        btVector3(24, 1, -20)
    };
    for (unsigned int i = 0; i < NUMBER_OF_OBSTACLES; ++i){
        obstacles.push_back(new Obstacle(obstaclesPositions[i], 2.0));
    }
    
    flock.CreateFlock(50, 20,boidObjects, obstacles);
    
}

void INM377ProjTemplateTorqueOrient::createBoids(){
    
    btScalar w = 3.0;//width
    btScalar h = 2.0;//height
    btScalar r = 3.0;//radius
    
    
    //create shape geometry structure
    //btCollisionShape* bShape = new btBoxShape(btVector3(5, 3, 5));
    std::vector<btVector3> parts = {
        btVector3(r, 0, 0),
        btVector3(0, h, 0),
        btVector3(0, 0, w),
        btVector3(0, 0, -w)
    };
    
    btVector3 tempAcc(0, 0, 0);
    for (unsigned int b = 0; b < NUMBER_OF_BOIDS; ++b){
        m_collisionShapes.push_back(flock.m_boids[b]->GetHullShape());
        btVector3 tempPos(rand() % 50, 0, rand() % 50);
        btVector3 tempVel(1, 0, 0);
        //btVector3 tempVel(rand() % 20, 0, rand() % 20);
        //btVector3 tempVel(0, rand() % 10, 0);
        
        //btScalar angle = rand() % int(btScalar(M_PI_2));
        //tempVel = btVector3(cos(angle), 0, sin(angle));
        
        
        flock.m_boids[b]->Set(parts, tempPos, tempVel, tempAcc, w, h, r, 1.0, 4.0, 10.0);
        
        //bind and create shape with mass, transform, and structure
        flock.m_boids[b]->m_body = localCreateRigidBody(flock.m_boids[b]->GetMass(), flock.m_boids[b]->GetTransform(), flock.m_boids[b]->GetHullShape());
        
        flock.m_boids[b]->Activate();
    }
    
}

void INM377ProjTemplateTorqueOrient::createObstacle(){
  
    for (unsigned int o = 0; o < NUMBER_OF_OBSTACLES; ++o){
        
        btCollisionShape* collisionShape = new btCylinderShape (btVector3(flock.m_obstacles[o]->getRadius(), 10.0, flock.m_obstacles[o]->getRadius()));
        //btCollisionShape* collisionShape = new btSphereShape(5.0);
        //btCollisionShape* collisionShape = new btBoxShape(btVector3(1.0, 10.0, 1.0));
        m_collisionShapes.push_back(collisionShape);
        
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(flock.m_obstacles[o]->getCentre());
        
        btScalar mass(50.0f);
        btVector3 cLocalInertia;
        collisionShape->calculateLocalInertia(mass, cLocalInertia);
        
        collisionBodies[o] = localCreateRigidBody(mass, trans, collisionShape);
        collisionBodies[o]->setAnisotropicFriction(collisionShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
        collisionBodies[o]->setFriction(0.5);
        //collisionBodies[i]->setLinearVelocity(btVector3(1, 0, 0));
        collisionBodies[o]->activate(true);
    }

    
}



// Apply steering forces (called on each iteration of the physics
// engine) including physical forces of flight, flocking behaviour
// and avoiding obstacles.
static void steer(btDynamicsWorld *world, const btScalar &timeStep){
    
//    std::vector<Boid*> boids = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boidObjects;
//   
    //static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.Run();
    
    
    
}

static btVector3 bvelocity(1,0,0);
static btVector3 bavelocity(0,0,0);
static btVector3 bacceleration(0,0,0);
static btVector3 bdesired(0,0,0);
static btScalar maxspeed = 10;
static btScalar maxforce = 7.0;
static btVector3 bsteer(0,0,0);
static btTransform btransform;
static btVector3 bposition(0,0,0);
static btVector3 btorqueTurnLeft(0,0,0);
static btVector3 btorqueTurnRight(0,0,0);

static btScalar bdistanceXp;
static btScalar bdistanceXn;
static btScalar bimpluseX;
static btScalar bdistanceYp;
static btScalar bdistanceYn;
static btScalar bimpluseY;
static btScalar bdistanceZp;
static btScalar bdistanceZn;
static btScalar bimpluseZ;
static btScalar bboundary = 100;
static btScalar bboundaryintersectiondistance = 10;



static btScalar mytrust = 20.0;
static btScalar myDrag = 0.99;
static btScalar myAngularDrag = 2.5;	


void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    //world->clearForces();
    
    //static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.Run();
    
    //Autonomous objects
    
    btRigidBody * bbody = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boid;
    btScalar bmass = bbody->getInvMass();
    btVector3 bgravity = bbody->getGravity();
    bposition = bbody->getCenterOfMassPosition();
    btransform = bbody->getWorldTransform();
    bavelocity = bbody->getAngularVelocity();
    btVector3 bdirection = bvelocity.safeNormalize();
    
    //btScalar angle = btAtan2(bdirection.x(), bdirection.y());
    //btScalar angleZ = - btAsin( (bdirection).z() );
    btQuaternion orientation = bbody->getOrientation();//orientation in 
    btVector3 baxisAngles = orientation.getAxis();
    btScalar bangle = orientation.getAngle();
    //orientation.setRotation(btVector3( 0.0, 0.0, 1.0 ), angle);
    //orientation.setRotation(btVector3( 0.0, 1.0, 0.0 ), angleZ);
    btMatrix3x3 bMatOrientation = btMatrix3x3(orientation); // quat to matrix
    //bbody->getWorldTransform().setBasis(bMatOrientation); 
    //bbody->setAngularVelocity(bavelocity);
    btVector3 bforward = btVector3(bMatOrientation[0][0], bMatOrientation[0][1], bMatOrientation[0][2]);
    btVector3 bback = bforward.normalize() * -1.0f;
    btVector3 bright = btVector3(bMatOrientation[2][0], bMatOrientation[2][1], bMatOrientation[2][2]);
    btVector3 bleft = bright.normalize() * -1.0f;
    btVector3 bup = btVector3(bMatOrientation[1][0], bMatOrientation[1][1], bMatOrientation[1][2]);
    btVector3 bdown = bup.normalize() * -1.0f;
    btVector3 bthrust = mytrust  * bforward ;//bfront
    btVector3 bdrag = -myDrag * bvelocity;//bvelocity;
    btVector3 bangulardrag = -myAngularDrag * bavelocity;//bavelocity
    
    btScalar maxSeeAhead = 20.0;
    btVector3 bahead = bposition + (bforward * maxSeeAhead);
    btVector3 bahead2 = bposition + (bforward * maxSeeAhead) * 0.5;
    
    btScalar distanceFromCenterPoint = 4.0;
    btVector3 momentArmLeft = (distanceFromCenterPoint * bup);
    btVector3 momentArmRight = (distanceFromCenterPoint * bdown);
    btVector3 forceOfSpin = -5.0 * (bavelocity);
    //btorqueTurnRight = (momentArmRight + forceOfSpin) ;
    
    btVector3 intersectPoint = btVector3(0,0,0);
    btScalar distanceFromIntersector = 0.0;
    btScalar distanceUntilIntersectorOccur = 0.0;
    btScalar xIntersect = 0;
    btScalar zIntersect = 0;
    
    bool isAhead1OutOfBounds = false;
    //bool isAhead2OutOfBounds = false;
    
    
    //going to far in negative x direction
    //going to far in positive x direction
    //going to far in negative z direction
    //going to far in positive z direction
   
    bdistanceXp = bboundary - bposition.x();
    bdistanceXn = bposition.x();
    bdistanceYp = bboundary - bposition.y();
    bdistanceYn = bposition.y();
    bdistanceZp = bboundary - bposition.z();
    bdistanceZn = bposition.z();
    
    if( bdistanceXp < bboundaryintersectiondistance){
        bimpluseX = bdistanceXp * 0.2;
        bbody->applyCentralImpulse(btVector3(bimpluseX,0,0));
    }else if (bdistanceXn < bboundaryintersectiondistance) { 
        bimpluseX = bdistanceXn * 0.2;
        bbody->applyCentralImpulse(btVector3(bimpluseX,0,0));
    }else{
        bimpluseX = 0;
        bbody->setAngularVelocity(btVector3(0,0,0));
    }
    
    if( bdistanceYp < bboundaryintersectiondistance){
        bimpluseY = bdistanceYp * 0.2;
        bbody->applyCentralImpulse(btVector3(0,bimpluseY,0));
    }else if( bdistanceYn < bboundaryintersectiondistance ) { 
        bimpluseY = bdistanceYn * 0.2;
        bbody->applyCentralImpulse(btVector3(0,bimpluseY,0));
    }else{
        bimpluseY = 0;
        
    }
    
    if( bdistanceZp < bboundaryintersectiondistance){
        bimpluseZ = bdistanceZp * 0.2;
        bbody->applyCentralImpulse(btVector3(0,0,bimpluseZ));
    }else if( bdistanceZn < bboundaryintersectiondistance ) { 
        bimpluseZ = bdistanceZn * 0.2;
        bbody->applyCentralImpulse(btVector3(0,0,bimpluseZ));
                bbody->applyTorque(btVector3(0,0,25));
    }else{
        bimpluseZ = 0;
    }
    
    bbody->setAngularVelocity(btVector3(0,0,0));
    
    
    /*
    if( (bahead.x() < -50.0) || (bahead.x() > 50.0) || (bahead.z() < -50.0) || (bahead.z() > 50.0) ) { 
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
//       bdesired = bdesired.normalize() * maxspeed;
//        
//       bsteer = bdesired - bvelocity;
//       bsteer = bsteer.normalize() * maxforce;
        
        
        //bbody->applyForce(bvelocity, bsteer);
        //applyForce(bsteer);
        
        //btorqueTurnLeft = (momentArmLeft * distanceUntilIntersectorOccur);
        //btorqueTurnRight = (momentArmRight + forceOfSpin) ;
       
        //bbody->applyTorque(btorqueTurnLeft);
        //angReset = true;
        //Angle between local forward vector and bird center of mass position
        btScalar angle = bforward.angle(btVector3(bbody->getCenterOfMassPosition().x(), 0.0f, bbody->getCenterOfMassPosition().z()));
        
        //Apply torque for short amount of time while under 0.5 Pi
        //if (angle < (M_PI*0.5)){
            btVector3 TorqueForce(0, 5.0, 0);
            //bbody->applyTorque(TorqueForce);
       //}
        
    }else{
        
//        if (angReset){
//            bbody->setAngularVelocity(btVector3(0,0,0) );
//            angReset = false;
//        }
    }
    */
    
    
    //bbody->setLinearVelocity(bthrust);
   // bbody->applyCentralForce(bthrust);
    
    //thrust
    //btVector3 worldThrust =  btVector3(mytrust, 0, 0);
    //btVector3 localThrust =  btVector3((btransform * worldThrust) - btransform.getOrigin());
    //bbody->applyCentralForce(localThrust);
    //bbody->applyCentralImpulse(localThrust);
    
    //Rotate according to thrust
    //btVector3 rotation = bforward.cross(localThrust);
    //bbody->applyTorque(rotation);
    
    
    //Drag:
    //bbody->applyCentralForce(bdrag);
    //bbody->applyTorque(bangulardrag);
    
    
    
    
    
    std::cout << "is seeing ahead1: " << isAhead1OutOfBounds << std::endl;
    std::cout << "velocity, x: " << bvelocity.x() << " y: " << bvelocity.y() << " z: " << bvelocity.z() << std::endl;
    std::cout << "angular velocity, x: " << bavelocity.x() << " y: " << bavelocity.y() << " z: " << bavelocity.z() << std::endl;
     
}

void	INM377ProjTemplateTorqueOrient::initPhysics()
{
	setTexturing(true);
	setShadows(false);
	setCameraDistance(50.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-2000, -2000, -2000);
	btVector3 worldMax(2000, 2000, 2000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
    m_dynamicsWorld->setGravity(btVector3(0, -9.8, 0));
	m_dynamicsWorld->setInternalTickCallback(MyTickCallback, static_cast<void *>(this), true);

    //std::cout << " init physics" << std::endl;
        
    createGround();
    
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

        //createBoids();
        //createObstacle();
		
         
	}
    
    
    //		btCollisionShape* bShape = new btBoxShape(btVector3(5, 3, 5));
    btConvexHullShape * bShape = new btConvexHullShape();
    bShape->addPoint(btVector3(10, 0, 0));
    bShape->addPoint(btVector3(0, 3, 0));
    bShape->addPoint(btVector3(0, 0, 5));
    bShape->addPoint(btVector3(0, 0, -5));
    
    //m_collisionShapes.push_back(bShape);
    btTransform btrans;
    btrans.setIdentity();
    //		btCollisionShape* bshape = m_collisionShapes[3];
    btVector3 bpos(20, 0, 0);
    btrans.setOrigin(bpos);
    btScalar bmass(1.0f);
    btVector3 bLocalInertia;
    bShape->calculateLocalInertia(bmass, bLocalInertia);
    boid = localCreateRigidBody(bmass, btrans, bShape);
    boid->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    boid->setFriction(0.5);
    //		boid->setLinearVelocity(btVector3(1, 0, 0));
    boid->activate(true);
    
     
}

void	INM377ProjTemplateTorqueOrient::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void INM377ProjTemplateTorqueOrient::keyboardCallback(unsigned char key, int x, int y)
{
	if (key=='p')
	{
		
		clientResetScene();
	} else
	{
		DemoApplication::keyboardCallback(key,x,y);
	}
}



void	INM377ProjTemplateTorqueOrient::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
    
	
}




