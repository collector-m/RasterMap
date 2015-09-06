#include "GLFunc.h"

extern RasterMap *rm;
extern Draw *m_draw;

int GLFunc::m_window_width = 1000;
int GLFunc::m_window_height = 1000;

int GLFunc::m_window_pos_x = 250;
int GLFunc::m_window_pos_y = 10;

float GLFunc::m_rotx = 0.0f;
float GLFunc::m_roty = 0.0f;
float GLFunc::m_rotz = 0.0f;

float GLFunc::m_tx = 0.0f;
//float GLFunc::m_ty = -500.0f;
float GLFunc::m_ty = 0.0f;
float GLFunc::m_tz = -3000.000f;
//float GLFunc::m_tz = -40000.000f;


int GLFunc::m_lastx = 0;
int GLFunc::m_lasty = 0;
unsigned char GLFunc::m_gl_buttons[3];
bool GLFunc::m_pause_screen = false;
int GLFunc::p_times = 10;

float GLFunc::shift = 100.0;
float GLFunc::click_x = 0.0;
float GLFunc::click_y = 0.0;
float GLFunc::last_click_x = 0.0;
float GLFunc::last_click_y = 0.0;

void GLFunc::Draw()
{
    glColor3ub(100, 100, 100);
    //DrawRasterMap();

    //DrawGlobalGrids();

    //DrawOutMap();

	///*
	DrawSubGrids();

	DrawDetectBox();
    DrawVehicle();
	DrawLeadLine();
    DrawLeadLinePoints();
    DrawStretchPoints();
	DrawWhiteGrids();
    DrawConvexGrids();
	//*/
    //DrawLine();
    //DrawLinePoints();
    //DrawObjects();
    //DrawBoxLines();

    //DrawStretchLines();
    //glColor3ub(255, 0, 0);

    //DrawRasterMap();


}

void GLFunc::Process(bool online)
{
    if(online) {
        rm->process(true, false);
        //usleep(10000);
        cout << "-----------------------------------------------------------------" << endl;
    } else {
        ProcessTest();
    }
}

void GLFunc::ProcessTest()
{
    int ugv_row = rm->ugv_pos.row;
    int ugv_col = rm->ugv_pos.col;
    if((ugv_row >= 0) && (ugv_row < OUTPUT_MAP_ROWS) && (ugv_col >= 0) && (ugv_col < OUTPUT_MAP_COLS)) {
        if((ugv_row >= rm->arr_dr) && (ugv_row <= rm->arr_ur) && (ugv_col >= rm->arr_lc) && (ugv_col <= rm->arr_rc))
            rm->g[ugv_row][ugv_col].test_occpy = false;
    }

//    if((click_x != 0) && (click_y != 0) && (click_x != last_click_x) && (click_y != last_click_y)) {
//        click_pt pt;
//        pt.x = click_x;
//        pt.y = click_y;
//        rm->pts.push_back(pt);
//    }

//    for(vector<click_pt>::iterator pt_iter = rm->pts.begin();
//            pt_iter != rm->pts.end(); pt_iter ++) {
//        int row = rm->getRow(pt_iter->x, pt_iter->y);
//        int col = rm->getCol(pt_iter->x, pt_iter->y);
//        cout << "click row: " << row << " click col: " << col << endl;
//        if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
//        if((row >= rm->arr_dr) && (row <= rm->arr_ur) && (col >= rm->arr_lc) && (col <= rm->arr_rc))
//            rm->g[row][col].test_occpy = true;
//        }
//    }

    int row = rm->getRow(click_x, click_y);
    int col = rm->getCol(click_x, click_y);
    cout << "click row: " << row << " click col: " << col << endl;
    if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
        if((row >= rm->arr_dr) && (row <= rm->arr_ur) && (col >= rm->arr_lc) && (col <= rm->arr_rc))
            rm->g[row][col].test_occpy = true;
    }

    last_click_x = click_x;
    last_click_y = click_y;
    rm->process(false, true);
    cout << "------------------------------------------" << endl;
}

void GLFunc::DrawObjects_old()
{
//    for(map<unsigned int, Object >::iterator map_iter = obs->convex_objs.begin();
//            map_iter != obs->convex_objs.end(); map_iter++) {
//        float xmin = map_iter->second.xmin;
//        float xmax = map_iter->second.xmax;
//        float ymin = map_iter->second.ymin;
//        float ymax = map_iter->second.ymax;
//        DrawBox(xmin, xmax, ymin, ymax);
//    }

//    for(vector<object_t>::iterator obj_iter = rm->objs.begin();
//            obj_iter != rm->objs.end(); obj_iter++) {
//        float xmin = obj_iter->xmin;
//        float xmax = obj_iter->xmax;
//        float ymin = obj_iter->ymin;
//        float ymax = obj_iter->ymax;
//        DrawBox(xmin, xmax, ymin, ymax);
//    }

}

void GLFunc::DrawObjects()
{
    for(map<unsigned int, object_t>::const_iterator map_iter = rm->objs.begin();
            map_iter != rm->objs.end(); map_iter++) {
		//if(ver_db.InDetectBox(local_objs[map_iter->first])) {
			float x1 = map_iter->second.bbx.vertex[0].x;
			float y1 = map_iter->second.bbx.vertex[0].y;
			float x2 = map_iter->second.bbx.vertex[1].x;
			float y2 = map_iter->second.bbx.vertex[1].y;
			float x3 = map_iter->second.bbx.vertex[2].x;
			float y3 = map_iter->second.bbx.vertex[2].y;
			float x4 = map_iter->second.bbx.vertex[3].x;
			float y4 = map_iter->second.bbx.vertex[3].y;
//        cout << " ( " << x1 << " , " << y1 << " ) "
//            << " ( " << x2 << " , " << y2 << " ) "
//            << " ( " << x3 << " , " << y3 << " ) "
//            << " ( " << x4 << " , " << y4 << " ) " << endl;
			DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
		//}
    }

//    for(map<unsigned int, object_t >::iterator map_iter = rm->objs.begin();
//            map_iter != rm->objs.end(); map_iter++) {
//        float xmin = map_iter->second.xmin;
//        float xmax = map_iter->second.xmax;
//        float ymin = map_iter->second.ymin;
//        float ymax = map_iter->second.ymax;
//        DrawBox(xmin, xmax, ymin, ymax);
//    }

}

void GLFunc::DrawBox(float xmin, float xmax, float ymin, float ymax)
{
    glColor3ub(139, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_QUADS);
        glVertex2f(p_times * xmin, p_times * ymin);
        glVertex2f(p_times * xmax, p_times * ymin);
        glVertex2f(p_times * xmax, p_times * ymax);
        glVertex2f(p_times * xmin, p_times * ymax);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
}

void GLFunc::DrawSubGrids()
{
    for(int i = 0; i <= 800; i++) {
        for(int j = 0; j <= 800; j++) {
            //unsigned int color = 255.0 - (int)(rm->g[i][j].global_p * 255.0);
            unsigned int color = 255 - (unsigned int)(rm->g[i][j].global_p * 255.0);
            glColor3ub(color, color, color);
            int row = i - 400;
            int col = j - 400;
            DrawOrthoGrid(row, col);

            if(rm->g[i][j].HaveObstacle) {
                glColor3ub(255, 255, 0);
                DrawOrthoGrid(row, col);
            }
        }
    }
}

void GLFunc::DrawOutMap()
{
    //for(int i = 0; i <= OUTPUT_MAP_ROWS; i++) {
    //    for(int j = 0; j <= OUTPUT_MAP_COLS; j++) {
	for(int i = OUTPUT_MAP_ROWS*3/8; i <= OUTPUT_MAP_ROWS*5/8; i++) {
        for(int j = OUTPUT_MAP_COLS*3/8; j <= OUTPUT_MAP_COLS*5/8; j++) {
            //draw the boundary info
			int row = i - OUTPUT_MAP_ROWS / 2;
            int col = j - OUTPUT_MAP_COLS / 2;

            unsigned char tpchar = rm->out_map[i][j].p[0];
            /*
			unsigned int color = 255 - (unsigned int)((rm->getProbability(tpchar)) * 255.0);
            glColor3ub(color, color, color);
            DrawOrthoGrid(row, col);
			*/
            int obs = rm->getObstacleAttr(tpchar);
            if(obs == 1) {
                glColor3ub(0, 255, 255);
                DrawOrthoGrid(row, col);
            } else if(obs == 2) {
                glColor3ub(255, 0, 0);
                DrawOrthoGrid(row, col);
            }

        }
    }
}

void GLFunc::DrawLine()
{
    double x1 = -250;
    double x2 = 250;
    double y1 = rm->sb.getY(x1);
    double y2 = rm->sb.getY(x2);
    //cout << x1 << " " << y1 << endl;
    //cout << x2 << " " << y2 << endl;

    glColor3ub(255, 255, 0);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
        glVertex2d(p_times * x1, p_times * y1);
        glVertex2d(p_times * x2, p_times * y2);
    glEnd();
	glLineWidth(1.0f);
}

void GLFunc::DrawLeadLine()
{
    glColor3ub(255, 255, 0);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
        glVertex2d(p_times * rm->lead_line.pt_down.x, p_times * rm->lead_line.pt_down.y);
        glVertex2d(p_times * rm->lead_line.pt_up.x, p_times * rm->lead_line.pt_up.y);
    glEnd();
    glLineWidth(1.0f);
}

void GLFunc::DrawGlobalGrids()
{
    for(int i = 0; i < GLOBAL_MAP_ROWS; i++) {
        for(int j = 0; j < GLOBAL_MAP_COLS; j++) {
            //unsigned int color = rm->g_map[i][j].p;
            //unsigned int color = (int)rm->g_map[i][j].p[0];
			unsigned int color = (unsigned int)rm->getProbability(rm->g_map[i][j].p[0]) * 255;
            glColor3ub(color, color, color);
            int row = i - GLOBAL_MAP_ROWS / 2;
            int col = j - GLOBAL_MAP_COLS / 2;
            DrawOrthoGrid(row, col);
        }
    }
}

void GLFunc::DrawWhiteGrids()
{
    for(int i = rm->arr_dr; i <= rm->arr_ur; i++) {
        for(int j = rm->arr_lc; j <= rm->arr_rc; j++) {
            if(rm->g[i][j].IsWhite) {
                int row = i - 400;
                int col = j - 400;
                glColor3ub(0, 120, 0);
                DrawOrthoGrid(row, col);
            }

        }
    }
}

void GLFunc::DrawVehicle()
{
    glColor3ub(0, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_QUADS);
        glVertex3f(p_times * rm->ugv_ld.x, p_times * rm->ugv_ld.y, 0);
        glVertex3f(p_times * rm->ugv_rd.x, p_times * rm->ugv_rd.y, 0);
        glVertex3f(p_times * rm->ugv_ru.x, p_times * rm->ugv_ru.y, 0);
        glVertex3f(p_times * rm->ugv_lu.x, p_times * rm->ugv_lu.y, 0);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
}

void GLFunc::DrawDetectBox()
{
    glColor3ub(0, 180, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS);
        glVertex3f(p_times * rm->box_ld.x, p_times * rm->box_ld.y, 0);
        glVertex3f(p_times * rm->box_rd.x, p_times * rm->box_rd.y, 0);
        glVertex3f(p_times * rm->box_ru.x, p_times * rm->box_ru.y, 0);
        glVertex3f(p_times * rm->box_lu.x, p_times * rm->box_lu.y, 0);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

}

void GLFunc::DrawHorDetectBox()
{
    glColor3ub(0, 180, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS);
        //glVertex3f(p_times * rm->horBox_ld.x, p_times * rm->horBox_ld.y, 0);
        //glVertex3f(p_times * rm->horBox_rd.x, p_times * rm->horBox_rd.y, 0);
        //glVertex3f(p_times * rm->horBox_ru.x, p_times * rm->horBox_ru.y, 0);
        //glVertex3f(p_times * rm->horBox_lu.x, p_times * rm->horBox_lu.y, 0);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void GLFunc::DrawLinePoints()
{
    glPointSize(5.0f);
    glColor3ub(255, 0, 0);
    glBegin(GL_POINTS);
        for(vector<Point>::const_iterator pt_iter = rm->line_pts.begin(); pt_iter != rm->line_pts.end();
                pt_iter ++) {
            glVertex3f(p_times * pt_iter->x, p_times * pt_iter->y, 0);
        }
        //glVertex2f(p_times * rm->current_pose.x, p_times * rm->current_pose.y);
    glEnd();
	glPointSize(1.0f);
}

void GLFunc::DrawLeadLinePoints()
{
    glPointSize(3.0f);
    glColor3ub(255, 0, 0);
    glBegin(GL_POINTS);
        for(vector<Point>::const_iterator pt_iter = rm->lead_line_pts.begin();
                pt_iter != rm->lead_line_pts.end(); pt_iter++) {
            glVertex2d(p_times * pt_iter->x, p_times * pt_iter->y);
        }
    glEnd();
    glPointSize(1.0f);
}

void GLFunc::DrawStretchPoints()
{
    glPointSize(3.0f);
    glColor3ub(255, 0, 255);
    glBegin(GL_POINTS);
        for(vector<Point>::const_iterator pt_iter = rm->stretch_pts.begin();
                pt_iter != rm->stretch_pts.end(); pt_iter++) {
            glVertex2d(p_times * pt_iter->x, p_times * pt_iter->y);
        }
    glEnd();
    glPointSize(1.0f);
}

void GLFunc::DrawConvexGrids()
{
    glColor3ub(255, 0, 0);
    for(vector<cluster_pos_t>::iterator iter = rm->convex_pos.begin();
            iter != rm->convex_pos.end(); iter++) {
        //cout << "before row " << iter->row << " col: " << iter->col << endl;
        //int row = rm->getLocalRow(iter->row, iter->col) - OUTPUT_MAP_ROWS / 2;
        //int col = rm->getLocalCol(iter->row, iter->col) - OUTPUT_MAP_COLS / 2;

        int row = rm->getLocalRow(iter->row + GLOBAL_MAP_ROWS / 2, iter->col + GLOBAL_MAP_COLS / 2) - OUTPUT_MAP_ROWS / 2;
        int col = rm->getLocalCol(iter->row + GLOBAL_MAP_ROWS / 2, iter->col + GLOBAL_MAP_COLS / 2) - OUTPUT_MAP_COLS / 2;
        //cout << "draw row: " << row << " draw col: " << col << endl;
        DrawOrthoGrid(row, col);
    }
}

void GLFunc::DrawOrthoGrid(int row, int col)
{
   glBegin(GL_QUADS);
        glVertex3f(p_times * (col-0.5) * 0.2, p_times * (row-0.5) * 0.2, 0);
        glVertex3f(p_times * (col+0.5) * 0.2, p_times * (row-0.5) * 0.2, 0);
        glVertex3f(p_times * (col+0.5) * 0.2, p_times * (row+0.5) * 0.2, 0);
        glVertex3f(p_times * (col-0.5) * 0.2, p_times * (row+0.5) * 0.2, 0);
   glEnd();
}

void GLFunc::DrawVertexBox(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
    glColor3ub(139, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_QUADS);
        glVertex3f(p_times * x1, p_times * y1, 0);
        glVertex3f(p_times * x2, p_times * y2, 0);
        glVertex3f(p_times * x3, p_times * y3, 0);
        glVertex3f(p_times * x4, p_times * y4, 0);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
}

void GLFunc::DrawRasterMap()
{
    glPointSize(0.1f);
    //draw the detect area
//    glBegin(GL_LINE_LOOP);
//        glColor3ub(200, 200, 200);
//        glVertex3f(-p_times * max_left, -p_times * max_back, 0);
//        glVertex3f(p_times * max_right, -p_times * max_back, 0);
//        glVertex3f(p_times * max_right, p_times * max_front, 0);
//        glVertex3f(-p_times * max_left, p_times * max_front, 0);
//    glEnd();
    glBegin(GL_LINES);
        //draw the rows
        //for(unsigned int i = 0; i < rm->rows; i++) {
        //    glColor3ub(100, 100, 100);
        //    double  y = rm->deltaLength * i;
        //    glVertex3f(p_times * -MAX_WIDTH/2, p_times * y, 0);
        //    glVertex3f(p_times * MAX_WIDTH/2, p_times * y, 0);
        //}
        //cout << "-MAX_WIDTH/2:" << MAX_WIDTH/2 << endl;
        //cout << "max_left:" << max_left << endl;

        //draw the front rows
        for(unsigned int i = 0; i <= 400; i++) {
            glColor3ub(100, 100, 100);
            double y = 0.2 * (i + 0.5);
            glVertex3f(-p_times * (80 + 0.5 * 0.2), p_times * y, 0);
            glVertex3f( p_times * (80 + 0.5 * 0.2), p_times * y, 0);
        }
        //draw the back rows
        for(unsigned int i = 0; i <= 400; i++) {
            glColor3ub(100, 100, 100);
            double y = -(0.2 * (i + 0.5));
            glVertex3f(-p_times * (80 + 0.5 * 0.2), p_times * y, 0);
            glVertex3f( p_times * (80 + 0.5 * 0.2), p_times * y, 0);
        }

        for(unsigned int i = 0; i <= 400; i++) {
             glColor3ub(100, 100, 100);
             double x = 0.2 * (i + 0.5);
             glVertex3f(p_times * x,  p_times * (80 + 0.5 * 0.2), 0);
             glVertex3f(p_times * x, -p_times * (80 + 0.5 * 0.2), 0);
        }
        //draw the left cols
        for(unsigned int i = 0; i <= 400; i++) {
             glColor3ub(100, 100, 100);
             double x = -(0.2 * (i + 0.5));
             glVertex3f(p_times * x,  p_times * (80 + 0.5 * 0.2), 0);
             glVertex3f(p_times * x, -p_times * (80 + 0.5 * 0.2), 0);
        }
    glEnd();
}

void GLFunc::DrawBoxLines()
{
    glColor3ub(0, 255, 255);
    glLineWidth(2.0f);
    glBegin(GL_LINES);


    for(vector<Line>::const_iterator line_iter = rm->box_lines.begin();
            line_iter != rm->box_lines.end(); line_iter++) {
        glVertex3f(p_times * line_iter->pt_down.x, p_times * line_iter->pt_down.y, 0);
        glVertex3f(p_times * line_iter->pt_up.x, p_times * line_iter->pt_up.y, 0);
    }
    glEnd();
}

void GLFunc::DrawStretchLines()
{
    glColor3ub(0, 255, 255);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
        for(vector<Line>::const_iterator line_iter = rm->stretch_lines.begin();
                line_iter != rm->stretch_lines.end(); line_iter++) {
            double x1 = -250.0;
            double y1 = line_iter->getY(x1);
            double x2 = 250.0;
            double y2 = line_iter->getY(x2);
            glVertex2d(p_times * x1, p_times * y1);
            glVertex2d(p_times * x2, p_times * y2);
        }
    glEnd();
    glLineWidth(1.0f);
}

void GLFunc::Initialize(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(GLFunc::m_window_width, GLFunc::m_window_height);
    glutInitWindowPosition(GLFunc::m_window_pos_x, GLFunc::m_window_pos_y);
    glutCreateWindow("RasterMap");
    /*
    for(unsigned int i = 0; i < rm->rings; i++) {
        float rou = rm->deltaRou * i;
        for(unsigned int j = 0; j < rm->grids; j++) {
            float theta = rm->deltaTheta * j;
            unsigned int pos = i * rm->rings + j;
            vertexArray[pos] = GLFunc::p_times * rou * cos(theta);
            vertexArray[pos + 1] = GLFunc::p_times * rou * sin(theta);
        }
    }
    */
}

void GLFunc::setOccupyGrids(int row, int col)
{
    rm->g[row][col].HaveLUXPoint = true;
}

void GLFunc::gl_init_graphics()
{
    glClearColor(0, 0, 0, 0.5);
    //glClearColor(255, 255, 255, 0.5);
    glClearDepth(1.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GLFunc::gl_draw_graphics()
{
    //cout << "Enter gl_draw_graphics" << endl;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glLoadIdentity();
    glTranslated(m_tx, m_ty, m_tz);

    glRotatef(m_rotx, 1, 0, 0);
    glRotatef(m_roty, 0, 1, 0);
    glRotatef(m_rotz, 0, 0, 1);

//    glBegin(GL_LINES);
//        glColor3ub( 88, 29, 29);  //dark red x axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(100000, 0, 0);
//        glColor3ub( 29, 88, 29);   //dark green  z axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(0, 0, 100000);
//        glColor3ub( 255, 240, 0);   //yellow  y axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(0, 100000, 0);
// 	glEnd();

    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
        Draw();
    glDisable(GL_BLEND);

    glColor3ub(255, 255, 0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    glLoadIdentity();
        //gluOrtho2D(-(GLdouble)m_window_width, (GLdouble)m_window_width, -(GLdouble)m_window_height, (GLdouble)m_window_height);
        //glScalef(1, -1, 1);
        glScalef(1, 1, 1);
        glTranslatef(0, -m_window_height, 0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
            //do something
            //
            //
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	// Show the new scene
	glutSwapBuffers();
}


void GLFunc::gl_mouse(int b, int s, int x, int y)
{
	m_lastx = x;
	m_lasty = y;
	switch(b)
	{
	case GLUT_LEFT_BUTTON:
		m_gl_buttons[0] = ((GLUT_DOWN == s) ? 1 : 0);
        cout << "x: " << x << " " << "y: " << y << endl;
		break;
	case GLUT_MIDDLE_BUTTON:
		m_gl_buttons[1] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	case GLUT_RIGHT_BUTTON:
		m_gl_buttons[2] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void GLFunc::gl_mouse2(int button, int state, int x, int y)
{
	m_lastx = x;
	m_lasty = y;

	GLint viewport[4];
	GLdouble mvmatrix[16], projmatrix[16];
	GLint realy;    //OpenGL y coordinate position
	GLdouble wx, wy, wz;    //return world x, y, z coordinates on 33000
	//GLdouble rx, ry, rz;    //return world x, y, z coordinates on 0
    //GLdouble tx, ty;

	switch(button)
	{
        case GLUT_LEFT_BUTTON:
            m_gl_buttons[0] = ((GLUT_DOWN == state) ? 1 : 0);
            //cout << "x: " << x << " " << "y: " << y << endl;

            if(state == GLUT_DOWN) {
                glGetIntegerv(GL_VIEWPORT, viewport);
                glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
                glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
                realy = viewport[3] - (GLint)y - 1;
                //cout << "Coordinates at cursor are ( " << x << " , " << realy << ")" << endl;

//                gluUnProject((GLdouble)x, (GLdouble)realy, 0.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
//                cout << "World coordinates at z = 0.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;
                gluUnProject((GLdouble)x, (GLdouble)realy, 1.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
                //cout << "World coordinates at z = 30000.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;

                //cout << "m_tx: " << m_tx << " m_ty: " << m_ty << " m_tz: " << m_tz << endl;
//                gluUnProject((GLdouble)m_tx, (GLdouble)m_ty, 1.0, mvmatrix, projmatrix, viewport, &tx, &ty, &tz);

                //compute the real x, y, z;
                //rz = 0;
                //rx = (fabs(m_tz) * wx / (30000 + fabs(m_tz))) / p_times;
                //ry = (fabs(m_tz) * wy / (30000 + fabs(m_tz))) / p_times;
//                rx = (wx * fabs(m_tz / wz)) / p_times;
//                ry = (wy * fabs(m_tz / wz)) / p_times;

//                rx = (wx * fabs(m_tz / wz) - m_tx) / p_times ;
//                ry = (wy * fabs(m_tz / wz) - m_ty) / p_times;

                click_x = (wx * fabs(m_tz / wz) - m_tx) / p_times;
                click_y = (wy * fabs(m_tz / wz) - m_ty) / p_times;

                cout << "click coordinates: " << click_x << " , " << click_y << ")" << endl;

//                cout << "World coordinates at z = 0.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;
//                cout << "mvMatrix: ";
//                for(int i = 0; i < 16; i++)
//                    cout << mvmatrix[i] << " " ;
//                cout << endl;
//                cout << "projMatrix: ";
//                for(int i = 0; i < 16; i++)
//                    cout << projmatrix[i] << " " ;
//
//                cout << endl;
            }
            break;
        case GLUT_MIDDLE_BUTTON:
            m_gl_buttons[1] = ((GLUT_DOWN == state) ? 1 : 0);
            break;
        case GLUT_RIGHT_BUTTON:
            m_gl_buttons[2] = ((GLUT_DOWN == state) ? 1 : 0);
            break;
        default:
            break;
	}
	glutPostRedisplay();
}


void GLFunc::gl_motion(int x,int y)
{
	int diffx = x - m_lastx;
	int diffy = y - m_lasty;
	m_lastx = x;
	m_lasty = y;

    //added by Guorun
    /*
    if(m_gl_buttons[0]) {
        m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
    } else if(m_gl_buttons[1]) {
        m_tz += 1000.0f;
    } else if(m_gl_buttons[2]) {
        m_tx += 15.0f * diffx;
        m_ty -= 15.0f * diffy;
    }
    */
    ///*
	if( m_gl_buttons[0] && m_gl_buttons[2] ) //transition
	{
		//m_zoom += 25.0f * diffy;
		m_tx += 25.0f * diffx;
        m_ty -= 25.0f * diffy;
	}
	else if( m_gl_buttons[0] ) //rotation
	{
		m_tx += 10.0f * diffx;
        m_ty -= 10.0f * diffy;
	}
	else if( m_gl_buttons[2] ) //transition
	{
		//m_tx += 85.0f * diffx;
		//m_ty -= 85.0f * diffy;
		m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
	}
    //*/
	glutPostRedisplay();
}

void GLFunc::gl_motion2(int x,int y)
{
	int diffx = x - m_lastx;
	int diffy = y - m_lasty;
	m_lastx = x;
	m_lasty = y;


	if( m_gl_buttons[0] && m_gl_buttons[2] ) //transition
	{
		//m_zoom += 25.0f * diffy;
		m_tx += 25.0f * diffx;
        m_ty -= 25.0f * diffy;
	}
	else if( m_gl_buttons[0] ) //rotation
	{
		m_tx += 10.0f * diffx;
        m_ty -= 10.0f * diffy;


	}
	else if( m_gl_buttons[2] ) //transition
	{
		//m_tx += 85.0f * diffx;
		//m_ty -= 85.0f * diffy;
		m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
	}
	glutPostRedisplay();
}


void GLFunc::gl_keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
        case '.':
            m_tz += 100.0f;
            cout << "bigger!" << endl;
            break;
        case ',':
            m_tz -= 100.0f;
            cout << "smaller!" << endl;
            break;
        case 'a':
            //rm->delta_x -= 0.8f;
            //rm->delta_x -= 80.f;
            rm->delta_pose.x -= shift;
            cout << "left!" << endl;
            break;
        case 'd':
            //rm->delta_x += 0.8f;
            //rm->delta_x += 80.f;
            rm->delta_pose.x += shift;
            cout << "right!" << endl;
            break;
        case 's':
            //rm->delta_y -= 0.8f;
            //rm->delta_y -= 80.f;
            rm->delta_pose.y -= shift;
            cout << "down!" << endl;
            break;
        case 'w':
            //rm->delta_y += 0.8f;
            //rm->delta_y += 80.f;
            rm->delta_pose.y += shift;
            cout << "up!" << endl;
            break;
        case 'k':
            rm->delta_pose.eulr -= 0.05f;
			//rm->modify_delta_pose.eulr -= 0.05f;
            cout << "Rotate: anti-clockwise!" << endl;
            break;
        case 'l':
            rm->delta_pose.eulr += 0.05f;
			//rm->modify_delta_pose.eulr += 0.05f;
            cout << "Rotate: clockwise!" << endl;
            break;
        case 'c':
            rm->clearGridsTest(0);
            break;
        case 27:
            exit(0);
        case 32:
            m_pause_screen = !m_pause_screen;
            break;
        default:
            break;
	}
	glutPostRedisplay();
}

void GLFunc::gl_resize_graphics(int width, int height)
{
	cout << "width: " << width << " height: " << height << endl;
	if (width == 0)
		height = 1;
	m_window_width = width;
	m_window_height = height;

	// Adjust graphics to window size
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//gluPerspective(45, ((double)width)/height, 0.05, 400000);
    gluPerspective(45, ((double)width)/height, 0.05, 400000.0);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void GLFunc::gl_when_idle()
{
	if (!(m_pause_screen)) {
		usleep(10000);
        Process(true);

        m_draw->drawUGV(rm->current_pose);
        m_draw->showGlobalMap();

        glutPostRedisplay();
	} else {
//        int row = rm->getRow(click_x, click_y);
//        int col = rm->getCol(click_x, click_y);
//        if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
//            if((row >= rm->arr_dr) && (row <= rm->arr_ur) && (col >= rm->arr_lc) && (col <= rm->arr_rc))
//                rm->g[row][col].test_occpy = true;
//        }
		//cout << "pause" << endl;
	}
}
