#include <unistd.h>

#include <memory>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
//#include <GL/glut.h>
#include <GL/freeglut.h> //glut.h extension for fonts

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

bool is_moving_point;
int moving_point_id;
pcl::PointXYZ moving_point;

bool adding_point;
pcl::PointXYZ new_point;

boost::shared_ptr<vector<int>> pointIndicies(new vector<int>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> pointTree(OPC_RESOLUTION);

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

    float ox = 5.0f * WIDTH / REAL_WIDTH;
    float oy = 5.0f * HEIGHT / REAL_HEIGHT;

    glPointSize(10.0);

    glLineWidth(1.0);
    Eigen::Vector3f bmin, bmax;
    for (pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Iterator voxel = pointTree.begin(); voxel != pointTree.end(); ++voxel) {
        pointTree.getVoxelBounds(voxel, bmin, bmax);
        glColor3f(0.75, 0.75, 0.75);
        glBegin(GL_LINE_LOOP);
            glVertex2f(bmin.x(), bmax.y());
            glVertex2f(bmin.x(), bmin.y());
            glVertex2f(bmax.x(), bmin.y());
            glVertex2f(bmax.x(), bmax.y());
        glEnd();
    }

    vector<pcl::PointXYZ> points;
    
    glBegin(GL_POINTS);
    glColor3f(1, 0, 0);
    if (adding_point) {
        glVertex3fv((float*)&new_point);
    }
    glColor3f(1, 0, 1);
    if (is_moving_point) {
        glVertex3fv((float*)&moving_point);
    }

    glColor3f(0, 1, 0);
    int curve_pid = 0;
    for (int pid = 0; pid < pointCloud->size(); pid++) {
        auto &p = pointCloud->at(pid);
        if (p.x > 9999)
            continue;

        float percent = (float)curve_pid / (float)points_per_curve;
        glColor3f(percent, 0, 1 - percent);
        glVertex3fv((float*)&p);
        points.push_back(p);
        curve_pid = points.size() % points_per_curve;
    }
    glEnd();

    glColor3f(0, 0, 0);
    glLineWidth(5.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_MAP1_VERTEX_3);
    int total_rounds = 1 + points.size() / points_per_curve;
    for (int round = 0; round < total_rounds; round++) {
        int point_count = points_per_curve;
        if (round + 1 == total_rounds)
            point_count = points.size() % points_per_curve;

        glMap1f(GL_MAP1_VERTEX_3, 0.0, 1.0, sizeof(pcl::PointXYZ) / sizeof(float), point_count, (float*)&points[round * points_per_curve]);
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

    glRasterPos2f(-0.75, 0.75);
    glColor3f(0, 0, 0);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (const unsigned char*)"Mode: WORLD");

    glFlush();
}

bool find_voxel(double radius, pcl::PointXYZ center, pcl::PointXYZ *point = nullptr, int *index = nullptr) {
    vector<int> indicies;
    vector<float> k_sqr_distances;
    pointTree.radiusSearch(center, radius, indicies, k_sqr_distances);
    if (indicies.size() == 0) {
        return false;
    }
    if (index != nullptr)
        *index = indicies[0];
    if (point != nullptr)
        *point = pointCloud->at(indicies[0]);
    return true;
}

void mouse(int button, int state, int x, int y) {
    float wx, wy;
    window2world(x, y, wx, wy);

    switch (button) {
        case GLUT_LEFT_BUTTON: // move existing point
            if (state == GLUT_DOWN) {
                if (pointTree.getLeafCount() > 0) {
                    double radius = 10.0 * min(WIDTH / REAL_WIDTH, HEIGHT / REAL_HEIGHT);
                    is_moving_point = find_voxel(radius, pcl::PointXYZ{wx, wy, 0.0}, &moving_point, &moving_point_id);
                    //if (is_moving_point) {
                    //    pointTree.deleteVoxelAtPoint(moving_point);
                    //    pointIndicies->erase(pointIndicies->begin() + moving_point_id);
                    //}
                }
            } else if (is_moving_point) {
                is_moving_point = false;
                //pointCloud->at(moving_point_id) = moving_point;
                //pointTree.addPointFromCloud(moving_point_id, pointIndicies);
            }
            break;

        case GLUT_RIGHT_BUTTON: // add new point
            if (state == GLUT_DOWN) {
                new_point.x = wx;
                new_point.y = wy;
                adding_point = true;
            } else {
                adding_point = false;
                if(!find_voxel(pointTree.getResolution(), new_point)) {
                    pointTree.addPointToCloud(new_point, pointCloud, pointIndicies);
                }
            }
            break;

        case GLUT_MIDDLE_BUTTON: // remove existing point
            if (state == GLUT_DOWN) {
                int tmp_point_id;
                pcl::PointXYZ tmp_point;
                double radius = 10.0 * min(WIDTH / REAL_WIDTH, HEIGHT / REAL_HEIGHT);
                if (find_voxel(radius, pcl::PointXYZ{wx, wy, 0.0}, &tmp_point, &tmp_point_id)) {
                    pointTree.deleteVoxelAtPoint(tmp_point);
                    pointCloud->at(tmp_point_id).x = 999999;
                    //pointIndicies->erase(pointIndicies->begin() + tmp_point_id);
                    // VERY SLOW!!!!!
                    remove_if(pointIndicies->begin(), pointIndicies->end(), [tmp_point_id](int id) -> bool {
                        return tmp_point_id == id;
                    });
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

    if (adding_point) {
        new_point.x = wx;
        new_point.y = wy;
        redraw = true;
    }

    if (is_moving_point) {
        pointTree.deleteVoxelAtPoint(moving_point);
        //pointIndicies->erase(pointIndicies->begin() + moving_point_id);

        moving_point.x = wx;
        moving_point.y = wy;
        redraw = true;

        pointCloud->at(moving_point_id) = moving_point;
        pointTree.addPointFromCloud(moving_point_id, /*pointIndicies*/ nullptr);
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
    }
}

int main(int argc, char *argv[]) {
    pointTree.setInputCloud(pointCloud, pointIndicies);

    //glewInit();

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

