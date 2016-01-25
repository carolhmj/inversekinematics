#include "glwidget.h"
#include <QTimerEvent>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <QDebug>
#include <iostream>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QObject>
#include <Qt>

using namespace std;

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{

}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0,0,0,1);

    connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    timer.start(1);

    this->root = new Joint(Eigen::Vector4f(0.0f,0.0f,0.0f,1.f), Eigen::Vector3f(1.,0.,0.));
    this->root->setCurrRotation(0,0,-30);


    Joint* j1 = new Joint(Eigen::Vector4f(0.0f,1.f,0.0f,1.f), Eigen::Vector3f(0.,1.,0.));
    j1->setCurrRotation(0,0,-30);
    this->root->addChild(j1);


    Joint* j2 = new Joint(Eigen::Vector4f(0.0f,1.f,0.0f,1.f), Eigen::Vector3f(0.,0.,1.));
    j2->setCurrRotation(0,0,-30);
    j1->addChild(j2);
//    cout << "j2 pos\n" << j2->getPositionGlobal() << endl;

//    Joint* j3 = new Joint(Eigen::Vector4f(0.0f,1.f,0.0f,1.f), Eigen::Vector3f(1.,1.,0.));
//    j3->setCurrRotation(0,0,30);
//    j2->addChild(j3);

    root->updateTransform();

    cout << root->getPositionGlobal() << endl;
    cout << "\n====\n";
    cout << root->getChildren()[0]->getPositionGlobal() << endl;
    cout << "\n====\n";
    cout << root->getChildren()[0]->getChildren()[0]->getPositionGlobal() << endl;
    cout << "\n====\n";

    projection = Projections::ortho(-5,5,-5,5,5,-5);
    //view = Projections::lookAt(Eigen::Vector3f(0.0f,0.0f,1.0f),Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,1.0f,0.0f));
    target << 0, 0, 0, 1;
    end << 0, 0, 0, 1;

    timer.start(16.6666);
}

void GLWidget::resizeGL(int w, int h)
{
    if(h<w) {
        glViewport((w-h)/2,0,h,h);
    }
    else {
        glViewport(0,(h-w)/2,w,w);
    }
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    Eigen::Vector4f endP = this->end;
    Eigen::Vector4f targetP = this->target;

    drawCircle(endP.head<3>(), 0.02, colorEnd);
    drawCircle(targetP.head<3>(), 0.02, colorTarget);

//    this->root->updateTransform();
//    this->root->draw(projection * view);
    //cout << "=====\n";
    //Kinematic::inverseKinematics(root->getChildren()[0]->getChildren()[0], Eigen::Vector4f(0.f,1.f,0.f,0.f), 0.001, 0.001);

}

void GLWidget::mousePressEvent(QMouseEvent *event)
{

}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
//    qDebug() << "mouse release!\n";
//    if (event->button() == Qt::RightButton){
//      Kinematic::inverseKinematics(this->root,end,target,1);
//    }
//    Eigen::Vector3f ttarget(std::sqrt(2)*std::sin(DEG2RAD(20)), 2.5 + std::sqrt(2)*std::cos(DEG2RAD(20)), 0);
//    cout << "ttarget: " << ttarget << "\n";
//    flush(cout);
//    //Kinematic::inverseKinematics(this->root,end,ttarget,1);
}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    qDebug() << "key event: " << event->key() << "\n";
    if (event->key() == Qt::Key_Space) {
        qDebug() << "inverse kinematics\n";
        //Kinematic::inverseKinematics(this->root, end.head<3>(), target.head<3>(), 1);
    }
}

void GLWidget::keyReleaseEvent(QKeyEvent *event)
{
    qDebug() << "key event: " << event->key() << "\n";
}

void GLWidget::drawCircle(Eigen::Vector3f center, float radius, float color[3])
{
    //qDebug() << "draw circle\n";
    glColor3f(color[0],color[1],color[2]);
    glBegin(GL_LINE_LOOP);
        for (int i=0; i < 360; i++) {
           float degInRad = DEG2RAD(i);
           glVertex2f(center[0] + cos(degInRad)*radius, center[1] + sin(degInRad)*radius);
        }
    glEnd();
    glColor3f(1,1,1);
}

