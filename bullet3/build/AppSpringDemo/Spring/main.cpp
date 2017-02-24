/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008
*/
#include "MySpringDemo.h"
#include "GlutStuff.h"

#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        //SliderConstraintDemo* sliderConstraintDemo = new SliderConstraintDemo();
		MySpringDemo* mySpringDemo = new MySpringDemo();
        mySpringDemo->initPhysics();
		mySpringDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
		mySpringDemo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
       

        return glutmain(argc, argv,640,480,"Slider Constraint Demo2. http://www.continuousphysics.com/Bullet/phpBB2/", mySpringDemo);
}

