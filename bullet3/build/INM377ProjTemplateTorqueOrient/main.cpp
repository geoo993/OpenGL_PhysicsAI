
#include <iostream>
#include "INM377ProjTemplateForcesTorqueOrient.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"


GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{
    
    INM377ProjTemplateTorqueOrient* demo = new INM377ProjTemplateTorqueOrient();
    srand ( time(nullptr) );
    
    demo->initPhysics();
    demo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    //demo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
    
    return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bulletphysics.com",demo);
    
}
