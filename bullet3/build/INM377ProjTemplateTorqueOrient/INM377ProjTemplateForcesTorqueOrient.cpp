
#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT 1.f

#include "INM377ProjTemplateForcesTorqueOrient.h"

#if 0
extern btAlignedObjectArray<btVector3> debugContacts;
extern btAlignedObjectArray<btVector3> debugNormals;
#endif 

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

#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();

	swapBuffers();

}


void INM377ProjTemplateTorqueOrient::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

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

//create gound in the scene
void INM377ProjTemplateTorqueOrient::CreateGround(){
    
    ///create a few basic rigid bodies
    btBoxShape* box = new btBoxShape(btVector3(btScalar(200.0),btScalar(1.0),btScalar(200.0)));
	box->initializePolyhedralFeatures();
    btCollisionShape* groundShape = box;
    
    m_collisionShapes.push_back(groundShape);
    m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,0));
    
    //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
    {
        btScalar mass(0.0);
        
        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.0);
        
        btVector3 localInertia(0,0,0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass,localInertia);
        
        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setFriction(0.5);
        
        body->setRollingFriction(0.3);
        //add the body to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
}

//set the boid and obstacle position and creating the flock
void INM377ProjTemplateTorqueOrient::InitialiseFlock(){
    
    std::vector<Boid*>	boidObjects;
    std::vector<Obstacle *> obstacles;
    
    boidObjects.reserve(NUMBER_OF_BOIDS);
    obstacles.reserve(NUMBER_OF_OBSTACLES);
    
    for (unsigned int a = 0; a < NUMBER_OF_BOIDS; ++a){
        boidObjects.push_back(new Boid);
    }
    
    std::vector<btVector3> obstaclesPositions = {
        btVector3(-5, 1, 40),
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
        btVector3(20, 1, -10),
        btVector3(15, 1, 25),
        btVector3(60, 1, 4),
        btVector3(-30, 1, 12),
        btVector3(-20, 1, 50),
        btVector3(-75, 1, 8),
        btVector3(-6, 1, 60),
        btVector3(40, 1, -22),
        btVector3(70, 1, -10)
    };
    for (unsigned int i = 0; i < NUMBER_OF_OBSTACLES; ++i){
        obstacles.push_back(new Obstacle(obstaclesPositions[i], 2.0));
    }
    
    flock.CreateFlock(boidObjects, obstacles);
    
}

//creating the boids and passing in their position and refering to boid in the list of boids
void INM377ProjTemplateTorqueOrient::CreateBoids(){
    
    for (unsigned long int b = 0; b < NUMBER_OF_BOIDS; ++b){
        btVector3 tempPos( Extension::randomFloatBetween(-30, 30), Extension::randomFloatBetween(2, 30), Extension::randomFloatBetween(-30, 30));//-30 and 30
        NewBoids(b, tempPos);
    }
    
}

//creating the obstacle and refering to obstacle in the list of obstacles
void INM377ProjTemplateTorqueOrient::CreateObstacle(){
  
    for (unsigned long int o = 0; o < NUMBER_OF_OBSTACLES; ++o){
        NewObstacle(o);
    }
}

//adding boid in the flock
void INM377ProjTemplateTorqueOrient::NewBoids(const unsigned long int &index, const btVector3 &position){
    
    flock.m_boids[index]->SetPosition(position);
    m_collisionShapes.push_back(flock.m_boids[index]->GetHullShape());
    
    //bind and create shape with mass, transform, and structure
    flock.m_boids[index]->m_body =
    localCreateRigidBody(flock.m_boids[ index]->bGet(Boid::BoidsValues::BMASS), flock.m_boids[index]->GetTransform(), flock.m_boids[index]->GetHullShape());
    flock.m_boids[index]->Activate();
}

//adding obsctacle in the scene
void INM377ProjTemplateTorqueOrient::NewObstacle(const unsigned long int &index){
    
    btCollisionShape* collisionShape = new btCylinderShape (
                                                            btVector3(flock.m_obstacles[index]->getRadius(), 
                                                                      50.0, 
                                                                      flock.m_obstacles[index]->getRadius())
                                                            );
    //btCollisionShape* collisionShape = new btSphereShape(5.0);
    //btCollisionShape* collisionShape = new btBoxShape(btVector3(1.0, 10.0, 1.0));
    m_collisionShapes.push_back(collisionShape);
    
    btRigidBody* body = nullptr;
    
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(flock.m_obstacles[index]->getCentre());
    
    btScalar mass(0.0);
    btVector3 cLocalInertia(0,0,0);
    collisionShape->calculateLocalInertia(mass, cLocalInertia);
    
    btMotionState* motionState = nullptr;
    body = new btRigidBody(mass, motionState, collisionShape, cLocalInertia);
    body = localCreateRigidBody(mass, trans, collisionShape);
    body->setAnisotropicFriction(collisionShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
    body->setFriction(0.5);
    body->activate(true);
    
}


void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    
    world->clearForces();
    
    //updating flock in each frame
    static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->flock.UpdateFlock();
    
}

void	INM377ProjTemplateTorqueOrient::initPhysics()
{
	setTexturing(true);
	setShadows(false);
	setCameraDistance(50.0f);

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

        btVector3 tempPos( Extension::randomFloatBetween(-30, 30), Extension::randomFloatBetween(2, 30), Extension::randomFloatBetween(-30, 30));//-30 and 30
        unsigned long int index = (flock.m_boids.size() - 1);
        NewBoids(index, tempPos);
        
    }else if (key=='o')
    {
        btVector3 tempPos(Extension::randomFloatBetween(-100, 100) , 1, Extension::randomFloatBetween(-100, 100) ); // -100 and 100
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

