

#include "INM377ProjTemplateForcesTorqueOrient.h"
#include "GLDebugDrawer.h"
GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{
    
    INM377ProjTemplateTorqueOrient* demo = new INM377ProjTemplateTorqueOrient();
    srand ( time(nullptr) );
    
    demo->initPhysics();
    demo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    //demo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
    
    return glutmain(argc, argv,780,560,"Bullet Physics Demo. http://bulletphysics.com",demo);
    
}
