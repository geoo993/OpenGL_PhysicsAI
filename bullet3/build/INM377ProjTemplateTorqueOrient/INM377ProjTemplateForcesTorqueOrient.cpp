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
    
    flock.CreateFlock(50.0, boidObjects, obstacles);
    
}

void INM377ProjTemplateTorqueOrient::createBoids(){
    
    btVector3 tempAcc(0, 0, 0);
    for (unsigned int b = 0; b < NUMBER_OF_BOIDS; ++b){
        m_collisionShapes.push_back(flock.m_boids[b]->GetHullShape());
        btVector3 tempPos(rand() % 50, 0, rand() % 50);
        btVector3 tempVel(rand() % 5, 0, rand() % 5);
        
        flock.m_boids[b]->Set(tempPos, tempVel, tempAcc);
        
        //bind and create shape with mass, transform, and structure
        flock.m_boids[b]->m_body = localCreateRigidBody(flock.m_boids[b]->bGet(Boid::BoidsValues::BMASS), flock.m_boids[b]->GetTransform(), flock.m_boids[b]->GetHullShape());
        
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


static btScalar bdistanceYp;
static btScalar bdistanceYn;
static btScalar bboundary = 50;
static btScalar bboundaryintersectiondistance = 10;


//WORLD SPACE VECTORS
const btVector3 bworldup(0, 1, 0);
const btVector3 bworldforward(0, 0, 1);
const btScalar bceilingForce = 25.0;
const btScalar bfloorforce = 25.0;
static btScalar bmaxspeed = 14;
static btScalar bmaxforce = 7.0;
static btScalar myDrag = 3.0;
static btScalar myAngularDrag = 5.0;
static btScalar torqueRotateBack = 2.0;


void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    world->clearForces();
    
    static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.Update();
    
    /*
    btRigidBody * bbody = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boid;
    
    
    btScalar bmass = bbody->getInvMass();
    btVector3 bgravity = bbody->getGravity();
    btVector3 bposition = bbody->getCenterOfMassPosition();
    btTransform btransform = bbody->getWorldTransform();
    btVector3 bvelocity = bbody->getLinearVelocity();
    btVector3 bavelocity = bbody->getAngularVelocity();
    btVector3 bdirection = bvelocity.safeNormalize();
    
    
    btQuaternion orientation = bbody->getOrientation();//orientation in 
    btMatrix3x3 bMatOrientation = btMatrix3x3(orientation); // quat to matrix
    btVector3 bforward = btVector3(bMatOrientation[0][0], bMatOrientation[0][1], bMatOrientation[0][2]);
    btVector3 bback = bforward.normalize() * -1.0f;
    btVector3 bright = btVector3(bMatOrientation[2][0], bMatOrientation[2][1], bMatOrientation[2][2]);
    btVector3 bleft = bright.normalize() * -1.0f;
    btVector3 bup = btVector3(bMatOrientation[1][0], bMatOrientation[1][1], bMatOrientation[1][2]);
    btVector3 bdown = bup.normalize() * -1.0f;
    btVector3 bthrust =  btVector3((btransform * btVector3(bmaxspeed, 0, 0)) - btransform.getOrigin());
    btVector3 bdrag = -(myDrag) * bbody->getLinearVelocity();//bvelocity;
    btVector3 bangulardrag = -(myAngularDrag) * bbody->getAngularVelocity();//bavelocity
    
    //////////////////////////
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
    btScalar angleFromLocal = Extension::getAngleBetweenTwoPoints(bforward.x()
                                                                  ,0,bforward.z(),
                                                                  0.0,0.0,0.0);

    btScalar maxSeeAhead = 20.0;
    btVector3 bahead = bposition + (bforward * maxSeeAhead);
    btVector3 bahead2 = bposition + (bforward * maxSeeAhead) * 0.5;
    

    //thrust
    bbody->applyCentralForce(bthrust);
    
    //drag
    bbody->applyCentralForce(bdrag);
    bbody->applyTorque(bangulardrag);

    //y axis
    bdistanceYp = bboundary - bposition.y();
    bdistanceYn = bposition.y();
    if( bdistanceYp < bboundaryintersectiondistance){
        bbody->applyCentralForce(btVector3(0, -bceilingForce, 0));
    }else if( bdistanceYn < bboundaryintersectiondistance ) { 
        bbody->applyCentralForce(btVector3(0, bfloorforce, 0));
    }

    if (bbody->getCenterOfMassPosition().length() > bboundary){
       
        btVector3 TorqueForce(0, torqueRotateBack, 0);
        if(angleFromWorld > 0.0 && angleFromWorld < 90.0 ){
            if( (angleFromLocal > 0.0 && angleFromLocal < 45.0) || (angleFromLocal <= 0.0 && angleFromLocal > -125.0) ){
                bbody->applyTorque(TorqueForce);
            }
        }else if(angleFromWorld >= 90.0 && angleFromWorld < 180.0 ){
            if( (angleFromLocal > 125.0 && angleFromLocal < 180.0) || (angleFromLocal <= -45.0 && angleFromLocal > -180.0) ){
                bbody->applyTorque(TorqueForce);
            }
        }else if(angleFromWorld < 0.0 && angleFromWorld > -90.0){
            if( (angleFromLocal > 0.0 && angleFromLocal < 125.0) || (angleFromLocal <= 0.0 && angleFromLocal > -45.0) ){
                bbody->applyTorque(TorqueForce);
            }
        }else if(angleFromWorld <= -90.0 && angleFromWorld > -180.0 ){
            if( (angleFromLocal > 45.0 && angleFromLocal < 180.0) || (angleFromLocal <= -125.0 && angleFromLocal > -180.0) ){
                bbody->applyTorque(TorqueForce);
            }
        }
    }
    */
    
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

        createBoids();
        createObstacle();
		
	}
    
    
    //		btCollisionShape* bShape = new btBoxShape(btVector3(5, 3, 5));
//    btConvexHullShape * bShape = new btConvexHullShape();
//    bShape->addPoint(btVector3(10, 0, 0));
//    bShape->addPoint(btVector3(0, 3, 0));
//    bShape->addPoint(btVector3(0, 0, 5));
//    bShape->addPoint(btVector3(0, 0, -5));
//    
//    //m_collisionShapes.push_back(bShape);
//    btTransform btrans;
//    btrans.setIdentity();
//    //		btCollisionShape* bshape = m_collisionShapes[3];
//    btVector3 bpos(1, 0, 0);
//    btrans.setOrigin(bpos);
//    btScalar bmass(1.0);
//    btVector3 bLocalInertia;
//    bShape->calculateLocalInertia(bmass, bLocalInertia);
//    boid = localCreateRigidBody(bmass, btrans, bShape);
//    boid->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
//    boid->setFriction(0.5);
//    //		boid->setLinearVelocity(btVector3(1, 0, 0));
//    boid->activate(true);
//    
    
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




