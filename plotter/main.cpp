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
#include <GL/glut.h>

using namespace std;

constexpr int WIDTH = 320;
constexpr int HEIGHT = 320;

struct point {
    float x, y;
};

vector<vector<point>> points;

void reshape(int width, int height) {
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WIDTH, HEIGHT, 0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void display() {
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0, 0, 0);

    for (auto &&ps : points) {
        glBegin(GL_LINE_STRIP);
        for (auto &&point : ps) {
            glVertex2f(point.x, point.y);
        }
        glEnd();
    }

    glFlush();
}

void readpoints() {
    string line;
    point p;
    vector<point> ps;

    while (getline(cin, line)) {
        if (line.length() == 0) {
            points.emplace_back(ps);
            ps.clear();
            glutPostRedisplay();
            continue;
        }
        stringstream ss(line);
        ss >> p.x >> p.y;
        ps.push_back(p);
    }
    if (ps.size() > 0) {
        points.emplace_back(ps);
        glutPostRedisplay();
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Plotter");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    thread readerThread(readpoints);

    glutMainLoop();

    readerThread.join();

    return 0;
}

