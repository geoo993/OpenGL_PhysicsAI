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

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

Added support for ODE sover
April 24, 2008
*/

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include <stdio.h> //printf debugging

#include "MyChainDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

void MyChainDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(26.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
//	wp->getSolverInfo().m_erp = 0.8;

	{ // floor
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-76,0));
		btRigidBody* groundBody;
		groundBody = localCreateRigidBody(0, groundTransform, groundShape);
	}

	{ // chain
		btScalar mass = btScalar(1.0);
		btTransform trans;
		trans.setIdentity();
		btCollisionShape* shape =
			new btCapsuleShapeX(btScalar(0.2), btScalar(0.6));
		m_collisionShapes.push_back(shape);
		btVector3 linkEnd(1, 0, 0);

		btVector3 anchor(0, 10, -5);
		const int numLinks = 20;
		btRigidBody* link[numLinks];
		for (int i = 0; i < numLinks; i++) {
			trans.setOrigin(anchor + (i+0.5)*linkEnd);
			link[i] = localCreateRigidBody(mass, trans, shape);
		}

		btVector3 leftEnd = linkEnd/2;
		btVector3 rightEnd = -linkEnd/2;
		m_dynamicsWorld->addConstraint(
			new btPoint2PointConstraint(*link[0], rightEnd),
			true);
		for (int i = 1; i < numLinks; i++)
			m_dynamicsWorld->addConstraint(
				new btPoint2PointConstraint(*link[i-1], *link[i], leftEnd, rightEnd),
				true);
	}
}

MyChainDemo::~MyChainDemo()
{
	//cleanup in the reverse order of creation/initialization
	int i;
	//removed/delete constraints
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}
	//remove the rigidbodies from the dynamics world and delete them
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
	//delete dynamics world
	delete m_dynamicsWorld;
	//delete solver
	delete m_constraintSolver;
	//delete broadphase
	delete m_overlappingPairCache;
	//delete dispatcher
	delete m_dispatcher;
	delete m_collisionConfiguration;
} // MyChainDemo::~MyChainDemo()

void MyChainDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
 	//during idle mode, just run 1 simulation step maximum
	int maxSimSubSteps = m_idle ? 1 : 1;
	if(m_idle)
	{
		dt = 1.0/420.f;
	}
	int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();
	bool verbose = false;
	if (verbose)
	{
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
	}
	renderme();
	glFlush();
	glutSwapBuffers();
} // MyChainDemo::clientMoveAndDisplay()

void MyChainDemo::displayCallback(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
	renderme();
	glFlush();
	glutSwapBuffers();
} // MyChainDemo::displayCallback()

void MyChainDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key) 
	{
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
}
