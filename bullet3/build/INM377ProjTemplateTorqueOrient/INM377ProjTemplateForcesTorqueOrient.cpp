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
    
    InitialiseFlock();
    
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.0));
}

INM377ProjTemplateTorqueOrient::~INM377ProjTemplateTorqueOrient()
{
    exitPhysics();
}

void INM377ProjTemplateTorqueOrient::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
        m_dynamicsWorld->stepSimulation(ms/1000000,0);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	//displayText();
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

void INM377ProjTemplateTorqueOrient::CreateGround(){
    
    ///create a few basic rigid bodies
    btBoxShape* box = new btBoxShape(btVector3(btScalar(110.),btScalar(1.),btScalar(110.)));
	box->initializePolyhedralFeatures();
    btCollisionShape* groundShape = box;
    
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

void INM377ProjTemplateTorqueOrient::InitialiseFlock(){
    
    std::vector<Boid*>	boidObjects;
    std::vector<Obstacle *> obstacles;
    
    boidObjects.reserve(NUMBER_OF_BOIDS);
    obstacles.reserve(NUMBER_OF_OBSTACLES);
    
    for (unsigned int a = 0; a < NUMBER_OF_BOIDS; ++a){
        boidObjects.push_back(new Boid);
    }
    
    std::vector<btVector3> obstaclesPositions = {
        btVector3(-5, 1, 45),
        btVector3(-60, 1, 20),
        btVector3(0, 1, -35),
        btVector3(34, 1, -50),
        btVector3(75, 1, 35),
        btVector3(50, 1, 60),
        btVector3(-70, 1, 32),
        btVector3(20, 1, -70),
        btVector3(-75, 1, -35),
        btVector3(-20, 1, -60),
        btVector3(50, 1, -62),
        btVector3(20, 1, -10)
    };
    for (unsigned int i = 0; i < NUMBER_OF_OBSTACLES; ++i){
        obstacles.push_back(new Obstacle(obstaclesPositions[i], 2.0));
    }
    
    flock.CreateFlock(50.0, boidObjects, obstacles);
    
}

void INM377ProjTemplateTorqueOrient::CreateBoids(){
    
    for (unsigned long int b = 0; b < NUMBER_OF_BOIDS; ++b){
        btVector3 tempPos(rand() % 50, rand() % 30, rand() % 50);
        NewBoids(b, tempPos);
    }
    
}

void INM377ProjTemplateTorqueOrient::CreateObstacle(){
  
    for (unsigned long int o = 0; o < NUMBER_OF_OBSTACLES; ++o){
        NewObstacle(o);
    }

}

void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    world->clearForces();
    
    static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.UpdateFlock();
    
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
        
    CreateGround();
    
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
        CreateBoids();
        CreateObstacle();
		
	}
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
    }else if (key=='b')
    {
        btRigidBody *bboid;
        flock.addBoid(bboid);

        btVector3 tempPos(rand() % 50, rand() % 30, rand() % 50);
        unsigned long int index = (flock.m_boids.size() - 1);
        NewBoids(index, tempPos);
        
    }else if (key=='o')
    {
        btVector3 tempPos((rand() % 160) - 80, 1, (rand() % 160) - 80 );
        flock.addObstacle( new Obstacle(tempPos, 2.0));
        unsigned long int index = (flock.m_obstacles.size() - 1);
        NewObstacle(index);
    }
    else
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


void INM377ProjTemplateTorqueOrient::NewBoids(const unsigned long int &index, const btVector3 &position){
   
    flock.m_boids[index]->Set(position);
    m_collisionShapes.push_back(flock.m_boids[index]->GetHullShape());
    
    //bind and create shape with mass, transform, and structure
    flock.m_boids[index]->m_body =
    localCreateRigidBody(flock.m_boids[ index]->bGet(Boid::BoidsValues::BMASS), flock.m_boids[index]->GetTransform(), flock.m_boids[index]->GetHullShape());
    flock.m_boids[index]->Activate();
}

void INM377ProjTemplateTorqueOrient::NewObstacle(const unsigned long int &index){
    
    
    btCollisionShape* collisionShape = new btCylinderShape (
                                                            btVector3(flock.m_obstacles[index]->getRadius(), 
                                                                      20.0, 
                                                                      flock.m_obstacles[index]->getRadius())
                                                            );
    //btCollisionShape* collisionShape = new btSphereShape(5.0);
    //btCollisionShape* collisionShape = new btBoxShape(btVector3(1.0, 10.0, 1.0));
    m_collisionShapes.push_back(collisionShape);
    
    btRigidBody* body = nullptr;
    
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(flock.m_obstacles[index]->getCentre());
    
    btScalar mass(1000.0);
    btVector3 cLocalInertia(0,0,0);
    collisionShape->calculateLocalInertia(mass, cLocalInertia);
    
    btMotionState* motionState = nullptr;
    body = new btRigidBody(mass, motionState, collisionShape, cLocalInertia);
    body = localCreateRigidBody(mass, trans, collisionShape);
    body->setAnisotropicFriction(collisionShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    body->setFriction(0.5);
    body->activate(true);
    
}


