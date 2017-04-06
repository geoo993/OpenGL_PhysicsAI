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
#ifndef BT_CCD_PHYSICS_DEMO_H
#define BT_CCD_PHYSICS_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

//#include "LinearMath/btAlignedObjectArray.h"

#include <iostream>

#include "MyFlockingDemo.h"
#include "Obstacle.h"
#include "Boid.h"
#include "Extension.h"

class btBroadphaseInterface;
class btCollisionShape;
class btCollisionDispatcher;
class btConstraintSolver;
class btDefaultCollisionConfiguration;
class Flock;

class INM377ProjTemplateTorqueOrient : public PlatformDemoApplication
{
	
private:
    
    const int NUMBER_OF_BOIDS = 30;
    const int NUMBER_OF_OBSTACLES = 12;
    void InitialiseFlock();
    void CreateGround();
    
    void CreateBoids();
    
    void CreateObstacle();
    
    void NewBoids(const unsigned long int &index, const btVector3 &position);
    
    void NewObstacle(const unsigned long int &index);
   
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
    
	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;


public:

    Flock flock;
    
    INM377ProjTemplateTorqueOrient();
    virtual~INM377ProjTemplateTorqueOrient();
    
	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	void displayText();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void displayCallback();
	virtual void clientResetScene();

	static DemoApplication* Create()
	{
		INM377ProjTemplateTorqueOrient* demo = new INM377ProjTemplateTorqueOrient;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	
};

#endif //BT_CCD_PHYSICS_DEMO_H

