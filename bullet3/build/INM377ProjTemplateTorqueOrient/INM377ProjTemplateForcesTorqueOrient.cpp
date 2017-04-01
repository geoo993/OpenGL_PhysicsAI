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
:m_ccdMode(USE_CCD)
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
		
		glRasterPos3f(xStart, yStart, 0);
		switch (m_ccdMode)
		{
		case USE_CCD:
			{
				sprintf(buf,"Predictive contacts and motion clamping");
				break;
			}
		case USE_NO_CCD:
			{
				sprintf(buf,"CCD handling disabled");
				break;
			}
		default:
			{
				sprintf(buf,"unknown CCD setting");
			};
		};

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
        obstacles.push_back(new Obstacle(obstaclesPositions[i], 1.0));
    }
    
    flock.CreateFlock(50, 20,boidObjects, obstacles);
    
}

void INM377ProjTemplateTorqueOrient::createBoids(){
    
    btScalar w = 2.0;//width
    btScalar h = 2.0;//height
    btScalar r = 6.0;//radius
    
    
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
        btVector3 tempVel(rand() % 20, 0, rand() % 20);
        //btVector3 tempVel(0, rand() % 10, 0);
        
        btScalar angle = rand() % int(btScalar(M_PI_2));
        tempVel = btVector3(cos(angle), 0, sin(angle));
        
        
        flock.m_boids[b]->Set(parts, tempPos, tempVel, tempAcc, w, h, r, 1.0, 0.4, 2.0);
        
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

void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    world->clearForces();
    
    //steer(world, timeStep);
    
    //    std::vector<Boid*> boids = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boidObjects;
    //   
    //static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.Run();
    
    btScalar bwidth = 5.0;
    btScalar bheight = 5.0;
    
    btRigidBody* bbody0 = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boid;
   
    btQuaternion orientation = bbody0->getOrientation();//orientation in 
    btVector3 baxisAngles = orientation.getAxis();
    btScalar bangle = orientation.getAngle();
    btMatrix3x3 bMatOrientation = btMatrix3x3(orientation); // quat to matrix
    
    btVector3 bforward = btVector3(bMatOrientation[0][0], bMatOrientation[0][1], bMatOrientation[0][2]);
    btVector3 bbackward = bforward.normalize() * -1.0f;
    btVector3 bup = btVector3(bMatOrientation[1][0], bMatOrientation[1][1], bMatOrientation[1][2]);
    btVector3 bdown = bup.normalize() * -1.0f;
    btVector3 bright = btVector3(bMatOrientation[2][0], bMatOrientation[2][1], bMatOrientation[2][2]);
    btVector3 bleft = bright.normalize() * -1.0f;
    
    btScalar bmass = bbody0->getInvMass();
    btVector3 bvel = bbody0->getLinearVelocity();
    btVector3 bgravity = bbody0->getGravity();
    btTransform btrans(orientation);
    btVector3 up(0, 1, 0);
    btVector3 btop = btrans * up;
    btVector3 bfront = btrans * btVector3(1, 0, 0);//forward vector of the boid
    btVector3 bdir = bvel.safeNormalize();
    btVector3 bright2 = bfront.cross(bdir);//get normal or left/right vector
    btVector3 avel = bbody0->getAngularVelocity();//the spin about an axis through the centre of mass
    btVector3 bAhead = bbody0->getCenterOfMassPosition() + (bforward * 10.0);
    btScalar btrustBalance = 5.0;
    
    
    
    
    btScalar maxHeight = 20.0;
    btScalar ground = 3.0;
    btScalar x = bbody0->getCenterOfMassPosition().x();
    btScalar y = bbody0->getCenterOfMassPosition().y();
    btScalar z = bbody0->getCenterOfMassPosition().z();
    btScalar balance = 0.0;
    //x directions
    //going to far in negative x direction
    //going to far in positive x direction
    //going to far in negative z direction
    //going to far in positive z direction
    if( (bAhead.x() < -50.0) || (bAhead.x() > 50.0) || (bAhead.z() < -50.0) || (bAhead.z() > 50.0) ) { 
        balance = 0.1;
        btrustBalance = 4.0;
        //reduce velocity
        //turn
    }else {
        balance = 0.0;
        btrustBalance = 2.0;
    }
    
    btScalar verticalDifference = maxHeight - y;
    btScalar verticalPercentageDifference = Extension::percentageWith(verticalDifference, ground, maxHeight);
    btScalar pressure = verticalPercentageDifference / 100.0;    // -1 to  1
    btVector3 blift = - (1.00 + pressure) * bgravity * bvel.length();
    
    btVector3 bthrust = btrustBalance * bfront;
    btVector3 bdrag = - 3 * bvel;
    
    
    //btVector3 btorqueOverTime = 2 * bfront.cross(bdir) - 5.0 * avel;
    btScalar distanceFromCenterPoint = 10.0;// the greater the value the more spin it will have, meaning the further away you apply angular speed from the center of mass the more it will spin
    btVector3 momentArm = (distanceFromCenterPoint * bdown);//The distance from the pivot point to the point where the force acts is called the moment arm, it is a vector
    btVector3 forceOfSpin = -1.0 * (avel);
    btVector3 btorqueOverTime = (momentArm + forceOfSpin) * balance;// 5 is the scalar difference of the force we should apply
    //torque is the measurement of how much a force acting on an object causes that object to rotate
    
   
    
    
    std::cout << std::endl;
    std::cout << "forward, x: " << bforward.x() << " y: " << bforward.y() << " z: " << bforward.z() << std::endl;
    std::cout << "front, x: " << bfront.x() << " y: " << bfront.y() << " z: " << bfront.z() << std::endl;
    std::cout << "ahead, x: " << bAhead.x() << " y: " << bAhead.y() << " z: " << bAhead.z() << std::endl;
    std::cout << "direction, x: " << bdir.x() << " y: " << bdir.y() << " z: " << bdir.z() << std::endl;
    
    std::cout << "velocity, x: " << bvel.x() << " y: " << bvel.y() << " z: " << bvel.z() << std::endl;
    std::cout << "angular velocity, x: " << avel.x() << " y: " << avel.y() << " z: " << avel.z() << std::endl;
    //std::cout << "angle: " << bangle << std::endl;
    std::cout << "angle, x: " << baxisAngles.x() << " y: " << baxisAngles.y() << " z: " << baxisAngles.z() << std::endl;
    
    std::cout << "position, x: " << x << " y: " << y << " z: " << z << std::endl;
    
    std::cout << "bthrust, x: " << bthrust.x() << " y: " << bthrust.y() << " z: " << bthrust.z() << std::endl;
    
    std::cout << "lift, x: " << blift.x() << " y: " << blift.y() << " z: " << blift.z() << std::endl;
    std::cout << "gravity, x: " << bgravity.x() << " y: " << bgravity.y() << " z: " << bgravity.z() << std::endl;
    std::cout << "y: " << y << ", vertical difference: " << verticalDifference << ", vertical percentage difference: " << verticalPercentageDifference << ", pressure: " << pressure << std::endl; 
    
   
    //bbody0->applyCentralForce(bthrust + blift + bgravity + bdrag);
    bbody0->applyCentralForce(bthrust + blift + bgravity );//force to apply before
    
    bbody0->applyTorque(btorqueOverTime);//force to apply before
    //bbody0->applyTorque(- 0.5 * bup);
    //bbody0->applyTorque(0.5 * btop.cross(up) - 5 * avel);
    
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
    //m_dynamicsWorld->setGravity(btVector3(0, -9.8, 0));
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
    
    m_collisionShapes.push_back(bShape);
    btTransform btrans;
    btrans.setIdentity();
    //		btCollisionShape* bshape = m_collisionShapes[3];
    btVector3 bpos(0, 0, 0);
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
		switch (m_ccdMode)
		{
			case USE_CCD:
			{
				m_ccdMode = USE_NO_CCD;
				break;
			}
			case USE_NO_CCD:
			default:
			{
				m_ccdMode = USE_CCD;
			}
		};
		clientResetScene();
	} else
	{
		DemoApplication::keyboardCallback(key,x,y);
	}
}


void	INM377ProjTemplateTorqueOrient::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();


		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_shootBoxShape);
		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setContactProcessingThreshold(1e30);

		///when using m_ccdMode, disable regular CCD
		if (m_ccdMode==USE_CCD)
		{
			body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
			body->setCcdSweptSphereRadius(0.4f);
		}
		
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




