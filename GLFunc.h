#ifndef GLFUNC_H_INCLUDED
#define GLFUNC_H_INCLUDED

#include <windows.h>
#include <GL/glut.h>
#include <cmath>
#include <unistd.h>

#include "RasterMap.h"
#include "Draw.h"

class GLFunc
{
private:
    //openGL window size
    static int m_window_width;
    static int m_window_height;

    //openGL window postion
    static int m_window_pos_x;
    static int m_window_pos_y;

    //initial view port
    static float m_rotx, m_roty, m_rotz;
    static float m_tx, m_ty, m_tz;

    static int m_lastx, m_lasty;
    static unsigned char m_gl_buttons[3];
    static bool m_pause_screen;
    static int p_times;

    //static GLfloat* vertexArray;

    //
    static float shift;

    //click x, y
    static float click_x, click_y;
    static float last_click_x, last_click_y;

public:
    static void Initialize(int argc, char** argv);
    static void gl_init_graphics();
    static void gl_resize_graphics(int width, int height);
    static void gl_draw_graphics();
    static void gl_mouse(int button, int state, int x, int y);
    static void gl_motion(int x, int y);
    static void gl_keyboard(unsigned char key, int x, int y);
    static void gl_when_idle();

    static void gl_mouse2(int button, int state, int x, int y);
    static void gl_motion2(int x, int y);

    static void Draw();

    static void Process(bool online = true);
    static void ProcessTest();

    static void DrawSubGrids();
    static void DrawGlobalGrids();
        static void DrawOrthoGrid(int row, int col);

    static void DrawConvexGrids();
    static void DrawRasterMap();
    static void DrawOutMap();
    static void DrawDetectBox();
    static void DrawHorDetectBox();
    static void DrawBoxLines();
    static void DrawVehicle();
    static void DrawLine();
    static void DrawLinePoints();
    static void DrawWhiteGrids();
    static void DrawLeadLine();
    static void DrawLeadLinePoints();
    static void DrawStretchPoints();
    static void DrawStretchLines();

    static void DrawObjects();
        static void DrawObjects_old();
        static void DrawBox(float xmin, float xmax, float ymin, float ymax);
        static void DrawVertexBox(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);


    static void setOccupyGrids(int row, int col);

};

#endif // GLFUNC_H_INCLUDED
