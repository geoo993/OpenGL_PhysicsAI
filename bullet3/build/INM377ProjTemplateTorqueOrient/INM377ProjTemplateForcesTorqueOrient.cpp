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
    boidObjects.reserve(NUMBER_OF_BOIDS);
    
    obstacles.reserve(NUMBER_OF_OBSTACLES);
    collisionBodies.reserve(NUMBER_OF_OBSTACLES);
    
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.));
}


void INM377ProjTemplateTorqueOrient::clientMoveAndDisplay()
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

void INM377ProjTemplateTorqueOrient::createBoids(){
    
    for (unsigned int a = 0; a < NUMBER_OF_BOIDS; ++a){
        boidObjects.push_back(new Boid);
    }
    
    //create shape geometry structure
    //		btCollisionShape* bShape = new btBoxShape(btVector3(5, 3, 5));
    std::vector<btVector3> parts = {
        btVector3(5, 0, 0),
        btVector3(0, 2, 0),
        btVector3(0, 0, 3),
        btVector3(0, 0, -3)
    };
    
    
    for (unsigned int b = 0; b < NUMBER_OF_BOIDS; ++b){
        m_collisionShapes.push_back(boidObjects[b]->GetHullShape());
        btVector3 tempPos(rand() % 50, 0, rand() % 50);
        //btVector3 tempVel(rand() % 20, 0, rand() % 20);
        btVector3 tempVel(0, rand() % 10, 0);
        
        boidObjects[b]->Set(parts, tempPos, tempVel, 5.0, 1.0, 0.4, 2.0);
        
        //bind and create shape with mass, transform, and structure
        boidObjects[b]->m_body = localCreateRigidBody(boidObjects[b]->GetMass(), boidObjects[b]->GetTrans(), boidObjects[b]->GetHullShape());
        
        boidObjects[b]->Activate();
    }
    
}

void INM377ProjTemplateTorqueOrient::createObstacle(){
    
    std::vector<btVector3> positions = {
        btVector3(-5, 1, 0),
        btVector3(-30, 1, 20),
        btVector3(0, 1, 35),
        btVector3(24, 1, -20)
    };
   
    for (unsigned int i = 0; i < NUMBER_OF_OBSTACLES; ++i){
        
        btCollisionShape* collisionShape = new btCylinderShape (btVector3(1.0, 10.0, 1.0));
        //btCollisionShape* collisionShape = new btSphereShape(5.0);
        //btCollisionShape* collisionShape = new btBoxShape(btVector3(1.0, 10.0, 1.0));
        m_collisionShapes.push_back(collisionShape);
        
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(positions[i]);
        
        btScalar mass(50.0f);
        btVector3 cLocalInertia;
        collisionShape->calculateLocalInertia(mass, cLocalInertia);
        
        obstacles.push_back(new SphereObstacle(positions[i], 1.0));
        
        collisionBodies[i] = localCreateRigidBody(mass, trans, collisionShape);
        collisionBodies[i]->setAnisotropicFriction(collisionShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
        collisionBodies[i]->setFriction(0.5);
        //collisionBodies[i]->setLinearVelocity(btVector3(1, 0, 0));
        collisionBodies[i]->activate(true);
    }

    
}
btVector3 INM377ProjTemplateTorqueOrient::collisionAvoidance(btRigidBody *actor){
    
    
    //always maintain prudent separation from their neighbors (collision avoidance/seperation)
    //Collision Avoidance: avoid collisions with nearby flockmates
    //Generally, one boid's awareness of another is based on the distance and direction of the offset vector between them.
    //Static collision avoidance and dynamic velocity matching are complementary.
    //Collision avoidance is the urge to steer a way from an imminent impact.
    //Static collision avoidance is based on the relative position of the flockmates and ignores their velocity.
    
    //Separation is the behavior that causes an agent to steer away from all of its neighbors.
    
    btVector3 c;
    int neighborCount = 0;
    btScalar neighborhoodSphericalZone = 100.0;// alos known as the neighbor radius
    
    //When a neighboring agent is found, the distance from the agent to the neighbor is added to the computation vector.
    
    for (unsigned int b = 0; b < boidObjects.size(); ++b){
        btRigidBody*otherActor = boidObjects[b]->m_body;
        
        if (otherActor != actor)
        {
            if (actor->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition()) < neighborhoodSphericalZone){
                btScalar tempX = otherActor->getCenterOfMassPosition().x() - actor->getCenterOfMassPosition().x();
                btScalar tempY = otherActor->getCenterOfMassPosition().y() - actor->getCenterOfMassPosition().y();
                btScalar tempZ = otherActor->getCenterOfMassPosition().z() - actor->getCenterOfMassPosition().z();
               
                c = btVector3(tempX, tempY, tempZ);
                neighborCount++;
            }
            
        }
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return c.absolute();
    }
    
    //The computation vector is divided by the corresponding neighbor count, but before normalizing, there is one more crucial step involved. The computed vector needs to be negated in order for the agent to steer away from its neighbors properly.
    c = btVector3(c.x()/neighborCount, c.y()/neighborCount, c.z()/neighborCount);
    c *= -1;
    
    return c;
}



btVector3 INM377ProjTemplateTorqueOrient::velocityMarching(btRigidBody *actor){
    
    //Alignment is a behavior that causes a particular agent to line up with agents close by.
    //the flock quickly becomes "polarized", its members heading in approximately the same direction at approximately the same speed (velocity marching or allignment)
    //Velocity Matching: attempt to match velocity with nearby flockmates
    //velocity matching is based only on velocity and ignores position. It is a predictive version of collision avoidance: if the boid does a good job of matching velocity with its neighbors, it is unlikely that it will collide with any of them any time soon. 
    btVector3 v;
    
    //in a huge flock spread over vast distances, an individual bird must have a localized and filtered perception of the rest of the flock. A bird might be aware of three categories: itself, it's two or three nearest neighbors, and the rest of the flock.
    //those close enough to be considered neighbors of the specified actor
    int neighborCount = 0;
    //The neighborhood is defined as a spherical zone of sensitivity centered at the boid's local origin.
    btScalar neighborhoodSphericalZone = 100.0;// alos known as the neighbor radius
    
    
    //If an agent is found within the radius, its velocity is added to the computation vector, and the neighbor count is incremented.
    
    for (unsigned int b = 0; b < boidObjects.size(); ++b){
        btRigidBody*otherActor = boidObjects[b]->m_body;
        
        if (otherActor != actor)
        {
            //if (Extension::distance(actor->getCenterOfMassPosition(), otherActor->getCenterOfMassPosition()) < neighborhoodSphericalZone){
            if (actor->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition()) < neighborhoodSphericalZone){
                btScalar tempX = v.x() + otherActor->getLinearVelocity().x();
                btScalar tempY = v.y() + otherActor->getLinearVelocity().y();
                btScalar tempZ = v.z() + otherActor->getLinearVelocity().z();
                v = btVector3(tempX, tempY, tempZ);
                neighborCount++;
            }
            
        }
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return v.absolute();
    }
    
    //Finally, we divide the computation vector by the neighbor count and normalize it (divide it by its length to get a vector of length 1), obtaining the final resultant vector.
    v = btVector3(v.x()/neighborCount, v.y()/neighborCount, v.z()/neighborCount);
    v.normalize();
    //v.safeNormalize();
    
    return v;
}

btVector3 INM377ProjTemplateTorqueOrient::flockCentering(btRigidBody *actor){
    
    //boids stay near one another (flock centering or cohesion)
    //Flock Centering: attempt to stay close to nearby flockmates
    //Flock centering makes a boid want to be near the center of the flock.
    //Cohesion is a behavior that causes agents to steer towards the "center of mass" - that is, the average position of the agents within a certain radius.
    
    btVector3 p;
    int neighborCount = 0;
    btScalar neighborhoodSphericalZone = 100.0;// alos known as the neighbor radius
    
    for (unsigned int b = 0; b < boidObjects.size(); ++b){
        btRigidBody*otherActor = boidObjects[b]->m_body;
        
        if (otherActor != actor)
        {
            //if (Extension::distance(actor->getCenterOfMassPosition(), otherActor->getCenterOfMassPosition()) < neighborhoodSphericalZone){
            if (actor->getCenterOfMassPosition().distance(otherActor->getCenterOfMassPosition()) < neighborhoodSphericalZone){
                btScalar tempX = p.x() + otherActor->getCenterOfMassPosition().x();
                btScalar tempY = p.y() + otherActor->getCenterOfMassPosition().y();
                btScalar tempZ = p.z() + otherActor->getCenterOfMassPosition().z();
                
                p = btVector3(tempX, tempY, tempZ);
                neighborCount++;
            }
            
        }
        
    }
    
    //If no neighbors were found, we simply return the zero vector (the default value of the computation vector).
    if (neighborCount == 0){
        return p.absolute();
    }
    
    //the computation vector is divided by the neighbor count, resulting in the position that corresponds to the center of mass. However, we don't want the center of mass itself, we want the direction towards the center of mass, so we recompute the vector as the distance from the agent to the center of mass. Finally, this value is normalized and returned.
    p = btVector3(p.x()/neighborCount, p.y()/neighborCount, p.z()/neighborCount);
    btVector3 tempP = btVector3(
                                p.x() - actor->getCenterOfMassPosition().x(), 
                                p.y() - actor->getCenterOfMassPosition().y(), 
                                p.z() - actor->getCenterOfMassPosition().z());
    tempP.normalize();
    
    return tempP;
}




// Apply steering forces (called on each iteration of the physics
// engine) including physical forces of flight, flocking behaviour
// and avoiding obstacles.
static void steer(btDynamicsWorld *world, const btScalar &timeStep){
    
    ///*
    std::vector<Boid*> boids = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->boidObjects;
   
    
    for (unsigned int b = 0; b < boids.size(); ++b){
        //The computational abstraction that combines process, procedure, and state is called an actor
        btRigidBody* actor = boids[b]->m_body;

        
        //std::vector<SphereObstacle *>& obstacles = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->obstacles;
        
        btVector3 p = actor->getCenterOfMassPosition();
        //std::cout << "x: "<< p.x() << ", y:" << p.y()<< " z:" << p.z() <<std::endl;
        
        //btVector3 separation = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->collisionAvoidance(actor);
        //btVector3 alignment = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->velocityMarching(actor);
        //btVector3 cohesion = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flockCentering(actor);
        //btVector3 tempVel = alignment + cohesion + separation;
        //tempVel.normalize();
        //actor->setLinearVelocity(tempVel);
        //actor->applyCentralForce(tempVel);
        
        //An acceleration requests is used to determine which way to steer the boid.
        //The easiest way to combine acceleration requests is to average them. Because of the included "strength" factors, this is actually a weighted average.
        //Prioritized acceleration allocation is based on a strict priority ordering of all component behaviors, hence of the consideration of their acceleration requests.
        //The magnitude of each request is measured and added into another accumulator.
        //This process continues until the sum of the accumulated magnitudes gets larger than the maximum acceleration value, which is a parameter of each boid.

        
        //btScalar bmass = body->getInvMass();
        btVector3 bvel = actor->getLinearVelocity();
        btVector3 bgravity = actor->getGravity() * 0.1;
        btVector3 bdir = btVector3(0, 1, 1);
        btTransform btrans(actor->getOrientation());
        btVector3 up(0, 1, 0);
        btVector3 btop = btrans * up;
        btVector3 front = btrans * btVector3(1, 0, 0);
        btVector3 bdir1 = bvel.safeNormalize();
        btVector3  avel = actor->getAngularVelocity(); 
        btVector3 bthrust = 3.5 * front; //move forward 
        btVector3 bdrag = - 4 * bvel; //resist movement forward
        btVector3 blift = - 2.0 * bgravity * bvel.length(); //pressure agains gravity
        //actor->applyCentralForce(bthrust + blift + bgravity + bdrag);
        actor->applyCentralForce(bthrust + bgravity);
        actor->applyTorque(2 * front.cross(bdir) - 5.0 * avel);
        actor->applyTorque(- 0.5 * up);
        actor->applyTorque(0.5 * btop.cross(up) - 5 * avel);
        
        
        
    }
    //*/
    
}

static void updateObstacles(btDynamicsWorld *world, const btScalar &timeStep){
    
   
    //std::vector<SphereObstacle*> collShapes = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->obstacles;
    
    
    
//    btRigidBody* body0 = static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->body000;
//    btScalar mass = body0->getInvMass();
//    btVector3 vel = body0->getLinearVelocity();
//    btVector3 gravity = body0->getGravity();
//    btVector3 dir = btVector3(0, 0, 1);
//    btVector3 thrust = 7.0 * dir;
//    btVector3 drag = -3 * vel;
//    btVector3 lift = - 0.5 * gravity * vel.length();
//    body0->applyCentralForce(thrust + lift + gravity + drag );
}

void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    world->clearForces();
    
    steer(world, timeStep);
    
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

        createBoids();
        createObstacle();
		
         
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




