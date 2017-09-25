#include <unistd.h>

#include <memory>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include <GL/gl.h>
#include <GL/glext.h>
//#include <GL/glut.h>
#include <GL/freeglut.h> //glut.h extension for fonts

#include "octree.h"

using namespace std;

constexpr float LEFT = -1.0f;
constexpr float RIGHT = +1.0f;

constexpr float BOTTOM = -1.0f;
constexpr float TOP = +1.0f;

constexpr float WIDTH = RIGHT - LEFT;
constexpr float HEIGHT = TOP - BOTTOM;

constexpr double OPC_RESOLUTION = WIDTH / 10000.0;

enum class mode {
    bezier,
    bspline,
    clipping,
    count,
};

const char *szmode[(int)mode::count] = {
    "Bezier",
    "B-spline",
    "Clipping",
};

mode current_mode;

float REAL_WIDTH;
float REAL_HEIGHT;

int curve_points = 10;
int points_per_curve;

octree::point adding_point;
octree::point moving_point;

octree bezier_cloud(OPC_RESOLUTION);
octree bspline_cloud(OPC_RESOLUTION);
octree clipping_cloud(OPC_RESOLUTION);

octree& get_octree_by_mode(mode m) {
    switch(m) {
        case mode::bezier: return bezier_cloud;
        case mode::bspline: return bspline_cloud;
        case mode::clipping: return clipping_cloud;
        default: throw "no such mode";
    }
}

void window2world(int winX, int winY, float &wx, float &wy) {
    wx = (winX / REAL_WIDTH) * WIDTH + LEFT;
    wy = TOP - (winY / REAL_HEIGHT) * WIDTH;
}

void reshape(int width, int height) {
    REAL_WIDTH = width;
    REAL_HEIGHT = height;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(LEFT, RIGHT, BOTTOM, TOP);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void display() {
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glPointSize(10.0);

    octree &active_octree = get_octree_by_mode(current_mode);

    glLineWidth(1.0);
    Eigen::Vector3f bmin, bmax;
    for (pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Iterator voxel = active_octree.cloudSearch.begin(); voxel != active_octree.cloudSearch.end(); ++voxel) {
        active_octree.cloudSearch.getVoxelBounds(voxel, bmin, bmax);
    //for (octree::iterator voxel = bspline_cloud.begin(); voxel != bspline_cloud.end(); ++voxel) {
        //bspline_cloud.get_voxel_bounds(voxel, bmin, bmax);

        glColor3f(0.75, 0.75, 0.75);
        glBegin(GL_LINE_LOOP);
            glVertex2f(bmin.x(), bmax.y());
            glVertex2f(bmin.x(), bmin.y());
            glVertex2f(bmax.x(), bmin.y());
            glVertex2f(bmax.x(), bmax.y());
        glEnd();
    }

    glBegin(GL_POINTS);
    glColor3f(0, 1, 1);
    if (adding_point.is_valid()) {
        glVertex3fv((float*)&adding_point.coord);
    }
    glColor3f(1, 0, 1);
    if (moving_point.is_valid()) {
        glVertex3fv((float*)&moving_point.coord);
    }
    glEnd();

    vector<pcl::PointXYZ> defrag_points;

    // Enable clipping planes
    //
    // Draw clipping plane points
    glBegin(GL_POINTS);
    auto tmp3 = clipping_cloud.get_points();
    for (auto p = tmp3.begin(); p != tmp3.end(); ++p) {
        glColor3f(0, 0, 0);
        glVertex3fv((float*)&p.coord);
        defrag_points.push_back(p.coord);
    }
    glEnd();

    int max_plane_count = 0;
    glGetIntegerv(GL_MAX_CLIP_PLANES, &max_plane_count);
    for (int i = 0; i < max_plane_count; i++) {
        glDisable(GL_CLIP_PLANE0 + i);
    }

    for (int i = 0; i < (int)defrag_points.size() - 1; i += 2) {
        auto point1 = defrag_points[i];
        auto point2 = defrag_points[i + 1];

        glColor3f(0, 1, 0);
        glBegin(GL_LINES);
            glVertex3fv((float*)&point1);
            glVertex3fv((float*)&point2);
        glEnd();

        float A = point2.x - point1.x;
        float B = point2.y - point1.y;
        float C = 0.0f;
        float D = -A * point1.x - B * point1.y - C * point1.z;

        GLdouble plane[4] = { A, B, C, D };
        glClipPlane(GL_CLIP_PLANE0 + i / 2, plane);
        glEnable(GL_CLIP_PLANE0 + i / 2);
    }

    // BEZIER
    defrag_points.clear();

    //// Draw bezier points
    glBegin(GL_POINTS);
    auto tmp = bezier_cloud.get_points();
    for (auto p = tmp.begin(); p != tmp.end(); ++p) {
        int curve_pid = p.index % points_per_curve;
        float percent = (float)curve_pid / (float)points_per_curve;
        glColor3f(percent, 0, 1 - percent);
        glVertex3fv((float*)&p.coord);
        defrag_points.push_back(p.coord);
    }
    glEnd();

    //// Draw bezier curves
    glLineWidth(5.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_MAP1_VERTEX_3);
    int total_rounds = 1 + defrag_points.size() / points_per_curve;
    for (int round = 0; round < total_rounds; round++) {
        int point_count = points_per_curve;
        if (round + 1 == total_rounds)
            point_count = defrag_points.size() % points_per_curve;

        glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, sizeof(pcl::PointXYZ) / sizeof(float), point_count, (float*)&defrag_points[round * points_per_curve]);
        glBegin(GL_LINE_STRIP);
            for(int i = 0; i <= curve_points; i++) {
                float percent = (float)i / (float)curve_points;
                glColor3f(percent, 0, 1 - percent);
                glEvalCoord1f(percent);
            }
        glEnd();
        //glMapGrid1f(curve_points, 0.0, 1.0);
        //glEvalMesh1(GL_POINT, 0, curve_points);
    }
    glDisable(GL_MAP1_VERTEX_3);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);


    defrag_points.clear();
    // Draw b-spline points
    glBegin(GL_POINTS);
    auto tmp2 = bspline_cloud.get_points();
    for (auto p = tmp2.begin(); p != tmp2.end(); ++p) {
        glColor3f(0, 0, 0);
        glVertex3fv((float*)&p.coord);
        defrag_points.push_back(p.coord);
    }
    glEnd();

    total_rounds = 1 + defrag_points.size() / points_per_curve;
    for (int round = 0; round < total_rounds; round++) {
        int point_count = points_per_curve;
        if (round + 1 == total_rounds)
            point_count = defrag_points.size() % points_per_curve;

        if (point_count < 2)
            break;

        GLfloat *knots = new GLfloat[point_count * 2];
        for (int i = 0; i < point_count; i++) {
            knots[i] = 0;
            knots[point_count + i] = 1;
        }
        GLUnurbs *nurbs = gluNewNurbsRenderer();
        gluBeginCurve(nurbs);
        gluNurbsCurve(nurbs, point_count * 2, knots, sizeof(pcl::PointXYZ) / sizeof(float), (float*)&defrag_points[round * points_per_curve], point_count, GL_MAP1_VERTEX_3);
        gluEndCurve(nurbs);
        gluDeleteNurbsRenderer(nurbs);
        delete[] knots;
    }

    glRasterPos2f(-0.75, 0.75);
    glColor3f(0, 0, 0);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (const unsigned char*)szmode[(int)current_mode]);

    glFlush();
}

void mouse(int button, int state, int x, int y) {
    float wx, wy;
    window2world(x, y, wx, wy);
    octree &ot = get_octree_by_mode(current_mode);

    switch (button) {
        case GLUT_LEFT_BUTTON: // move existing point
            if (state == GLUT_DOWN) {
                double radius = 10.0 * min(WIDTH / REAL_WIDTH, HEIGHT / REAL_HEIGHT);
                moving_point = ot.find_point(radius, pcl::PointXYZ{wx, wy, 0.0});
            } else if (moving_point.is_valid()) {
                moving_point.invalidate();
            }
            break;

        case GLUT_RIGHT_BUTTON: // add new point
            if (state == GLUT_DOWN) {
                adding_point = octree::point(wx, wy);
                adding_point.validate();
            } else {
                if(!ot.find_point(-1, adding_point.coord).is_valid()) {
                    ot.add_point(adding_point);
                }
                adding_point.invalidate();
            }
            break;

        case GLUT_MIDDLE_BUTTON: // remove existing point
            if (state == GLUT_DOWN) {
                double radius = 10.0 * min(WIDTH / REAL_WIDTH, HEIGHT / REAL_HEIGHT);
                auto tmp_point = ot.find_point(radius, pcl::PointXYZ{wx, wy, 0.0});
                if (tmp_point.is_valid()) {
                    ot.delete_point(tmp_point);
                }
            }
            break;
    }

    glutPostRedisplay();
}

void motion(int x, int y) {
    bool redraw = false;
    float wx, wy;
    window2world(x, y, wx, wy);

    if (adding_point.is_valid()) {
        adding_point.coord.x = wx;
        adding_point.coord.y = wy;
        redraw = true;
    }

    if (moving_point.is_valid()) {
        octree &ot = get_octree_by_mode(current_mode);
        ot.move_point(pcl::PointXYZ{wx, wy, 0.0f}, moving_point);
        redraw = true;
    }

    if (redraw) {
        glutPostRedisplay();
    }
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case '<':
        case ',':
            curve_points--;
            cout << "Curve points = " << curve_points << endl;
            glutPostRedisplay();
            break;

        case '>':
        case '.':
            curve_points++;
            cout << "Curve points = " << curve_points << endl;
            glutPostRedisplay();
            break;

        case '[':
        case '{':
            points_per_curve--;
            cout << "Points per cruve = " << points_per_curve << endl;
            glutPostRedisplay();
            break;

        case ']':
        case '}':
            points_per_curve++;
            cout << "Points per curve = " << points_per_curve << endl;
            glutPostRedisplay();
            break;

        case 'm':
            current_mode = (mode)(((int)current_mode + 1) % (int)mode::count);
            glutPostRedisplay();
            break;
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Curve");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glGetIntegerv(GL_MAX_EVAL_ORDER, &points_per_curve);

    glutMainLoop();

    return 0;
}

