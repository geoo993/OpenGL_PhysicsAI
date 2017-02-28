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

#ifndef MY_SPRING_DEMO_H
#define MY_SPRING_DEMO_H

#include "btBulletDynamicsCommon.h"
#include "GlutDemoApplication.h"
#include <vector>

class Spring {
	btRigidBody* bodyA;
	btRigidBody* bodyB;
	btScalar length;
	btScalar hooke_constant;
	btScalar damping;

public:
	Spring(btRigidBody* a, btRigidBody* b, btScalar hooke, btScalar damp);
	void applyForce();
	void draw();
};

class SpringNetwork {
	std::vector<Spring> spring;
	btScalar hooke_constant;
	btScalar damping;

public:
	SpringNetwork(btScalar hooke, btScalar damp);
	void addSpring(btRigidBody* a, btRigidBody* b);
	void applyForce();
	void draw();
};

/// MySpringDemo shows how to create a slider constraint
class MySpringDemo : public GlutDemoApplication
{
	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	SpringNetwork springs;

	public:
	MySpringDemo();

	virtual ~MySpringDemo();

	void	initPhysics();

	void	initModel();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		MySpringDemo* demo = new MySpringDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}	

	virtual void keyboardCallback(unsigned char key, int x, int y);
};

#endif //MY_SPRING_DEMO_H


#if 0
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



#ifndef SLIDER_CONSTRAINT_DEMO_H
#define SLIDER_CONSTRAINT_DEMO_H



#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "GlutDemoApplication.h"




/// SliderConstraintDemo shows how to create a slider constraint
class SliderConstraintDemo : public GlutDemoApplication
{
	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	public:

	virtual ~SliderConstraintDemo();

	void	initPhysics();

	void	initModel();

	void	drawSliders();
	void	drawSliderConstraint(btSliderConstraint* constraint);


	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		SliderConstraintDemo* demo = new SliderConstraintDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}	

	virtual void keyboardCallback(unsigned char key, int x, int y);
	
};

#endif //SLIDER_CONSTRAINT_DEMO_H

#endif