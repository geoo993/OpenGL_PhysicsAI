
#ifndef BT_CCD_PHYSICS_DEMO_H
#define BT_CCD_PHYSICS_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif


#include <iostream>

#include "Boid.h"
#include "MyFlockingDemo.h"
#include "Obstacle.h"
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
    
    //create gound in the scene
    void CreateGround();
    
    //number of boids to start with
    const int NUMBER_OF_BOIDS = 100;
    
    //number of obstacles to start with
    const int NUMBER_OF_OBSTACLES = 20;
    
    //set the boid and obstacle position and creating the flock
    void InitialiseFlock();
    
    //creating the boids and passing in their position and refering to boid in the list of boids
    void CreateBoids();
    
    //creating the obstacle and refering to obstacle in the list of obstacles
    void CreateObstacle();
    
    //adding boid in the flock
    void NewBoids(const unsigned long int &index, const btVector3 &position);
    
    //adding obsctacle in the scene
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

