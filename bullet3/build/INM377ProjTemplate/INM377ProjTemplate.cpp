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

#include "INM377ProjTemplate.h"
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


INM377ProjTemplate::INM377ProjTemplate()
:m_ccdMode(USE_CCD)
{
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.));
}


void INM377ProjTemplate::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	//float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1./60.,0);//ms / 1000000.f);
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


void INM377ProjTemplate::displayText()
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



void INM377ProjTemplate::displayCallback(void) {

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

void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
		//world->clearForces();
		static_cast<btRigidBody *>(world->getWorldUserInfo())->applyGravity();
		static_cast<btRigidBody *>(world->getWorldUserInfo())->applyCentralForce(btVector3(0, 10, 0));
}

void	INM377ProjTemplate::initPhysics()
{
	setTexturing(true);
	setShadows(false);

	setCameraDistance(26.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
	m_dynamicsWorld->setInternalTickCallback(MyTickCallback, static_cast<void *>(&body000), true);
	

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
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction(0.5);

		//body->setRollingFriction(0.3);
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

//		btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		
//		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
//		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
//		btTransform startTransform;
//		startTransform.setIdentity();

//		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
//		bool isDynamic = (mass != 0.f);

//		btVector3 localInertia(0,0,0);
//		if (isDynamic)
//			colShape->calculateLocalInertia(mass,localInertia);

        
        
        
//        btCollisionShape* colShape = new btBoxShape(btVector3(5, 5, 5));
//        m_collisionShapes.push_back(colShape);
//        btTransform trans;
//        trans.setIdentity();
//        btCollisionShape* shape = m_collisionShapes[2];
//        btVector3 pos(0, 6, 0);
//        trans.setOrigin(pos);
//        btScalar mass(0.0f);
//        // shape->calculateLocalInertia(mass, LocalInertia);
//        body000 = localCreateRigidBody(mass, trans, shape);
//        body000->setAnisotropicFriction(shape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
//        body000->setFriction(0.5);
//        //		body000->setLinearVelocity(btVector3(1, 0, 0));
//        body000->activate(true);
//        
//        
//        
//        //btVector3(5, 5, 5
//        btCollisionShape* linkShape = new btCapsuleShape(btScalar(1.0f),btScalar(3.0f));
//        m_collisionShapes.push_back(linkShape);
//        btTransform trans1;
//        trans1.setIdentity();
//        btVector3 pos1(10, 10, 0);
//        btVector3 anchor1(0, 2.5, 0);
//        
//        
//        trans1.setOrigin(pos1);
//        btScalar mass1(10.0f);
//        btVector3 LocalInertia1(0, 0, 0);
//        linkShape->calculateLocalInertia(mass1, LocalInertia1);
//        btRigidBody *link1 = localCreateRigidBody(mass1, trans1, linkShape);
//        //link1->setAnisotropicFriction(shape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
//        //link1->setFriction(0.5);
//        //		body000->setLinearVelocity(btVector3(1, 0, 0));
//        link1->activate(true);
//        btPoint2PointConstraint *joint1 = new btPoint2PointConstraint(*link1, anchor1);
//        m_dynamicsWorld->addConstraint(joint1, true);
//        
//        trans1.setOrigin(pos1 - anchor1);
//        btRigidBody *link2 = localCreateRigidBody(mass1, trans1, linkShape);
//        link2->activate(true);
//        
//        btPoint2PointConstraint *joint2 = new btPoint2PointConstraint(*link1, *link2, -anchor1, anchor1);
//        m_dynamicsWorld->addConstraint(joint2, true);
        
        
        btScalar mass = btScalar ( 1.0f) ;
        btTransform trans ;
        trans.setIdentity ( ) ;
        btCollisionShape * shape = new btCapsuleShapeX( btScalar ( 0.2f ), btScalar( 0.6f) ) ;
        m_collisionShapes . push_back(shape) ;
        btVector3 linkEnd( 1 , 0 , 0 ) ;
        btVector3 anchor( 0 , 20 , 0.0f) ;
        const int numLinks = 20;
        btRigidBody * link [ numLinks ] ;
        for ( int i = 0; i < numLinks; i++) {
            trans.setOrigin(anchor + (i+ 0.5f ) * linkEnd) ;
            link [ i ] = localCreateRigidBody(mass , trans , shape) ;
        }
        
        btVector3 leftEnd = linkEnd / 2 ; //camera is behind
        btVector3 rightEnd = - linkEnd / 2 ;
        m_dynamicsWorld->addConstraint( new btPoint2PointConstraint(*link[0] , rightEnd), true ) ;
        for ( int i = 1; i < numLinks ; i++){
            m_dynamicsWorld->addConstraint(new btPoint2PointConstraint( *link [ i - 1], *link [i], leftEnd,rightEnd), true ) ;
        }
        
        
    }
    
}

void	INM377ProjTemplate::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void INM377ProjTemplate::keyboardCallback(unsigned char key, int x, int y)
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


void	INM377ProjTemplate::shootBox(const btVector3& destination)
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




void	INM377ProjTemplate::exitPhysics()
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
	
//	delete m_solver;
	
//	delete m_broadphase;
	
//	delete m_dispatcher;

//	delete m_collisionConfiguration;

	
}




