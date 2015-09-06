#include <iostream>

#include "RasterMap.h"
#include "Draw.h"
#include "GLFunc.h"

RasterMap *rm = NULL;
Draw *m_draw = NULL;

using namespace std;

int main(int argc, char** argv)
{
    rm = new RasterMap();
    rm->readGlobalMap();
    rm->getSHMBasePoint();

    m_draw = new Draw();
    m_draw->readGlobalMap("D:/run/ugv/RasterMap/map_visualize.png");
    m_draw->setGlobalMapCenter(rm->dgps_pose);



    //visualization version
	///*
    GLFunc::Initialize(argc, argv);
	glutDisplayFunc(GLFunc::gl_draw_graphics);
	glutReshapeFunc(GLFunc::gl_resize_graphics);
	glutMouseFunc(GLFunc::gl_mouse2);
	glutMotionFunc(GLFunc::gl_motion2);
	glutKeyboardFunc(GLFunc::gl_keyboard);
	GLFunc::gl_init_graphics();
	glutIdleFunc(GLFunc::gl_when_idle);
	glutMainLoop();
	//*/
//	//no visualization version
//	///*
//	while(1) {
//		rm->process(true, false);
//		cout << "------------------------------------" << endl;
//	}
//	//*/

    delete rm;
    return 0;
}
