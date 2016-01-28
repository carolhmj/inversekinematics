#include "glwidget.h"
#include <QTimerEvent>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <QDebug>
#include <iostream>
#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <QObject>
#include <Qt>

using namespace std;

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    //qDebug() << "connecttest: " << QObject::connect(&timer,SIGNAL(timeout()),this,SLOT(update())) << "\n";
    connect(&timer,SIGNAL(timeout()),this,SLOT(update()));
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0,0,0,1);
    this->root = new Joint(Eigen::Vector3f(0.0f,0.0f,0.0f));

    std::vector<Eigen::Vector4f> ldata1;
    Eigen::Vector4f ldata1center(0,1,0,1);

    ldata1.push_back(Eigen::Vector4f(0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,2,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,2,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,2,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,2,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,0,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,0,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,0,-0.5,1));

    Link *l = new Link(ldata1,ldata1center);
    this->root->setLink(l);
    //this->root->setCurrRotation(0,0,45);
    //this->root->setCurrRotation(90,0,0);
    this->root->setCurrRotation(0,0,1,10);

    Joint* j1 = new Joint(Eigen::Vector3f(0.0f,1.0f,0.0f));
    //j1->setCurrRotation(0,0,30);
    j1->setCurrRotation(0,1,1,85);
    this->root->addChild(j1);
    Link *l1 = new Link(ldata1,ldata1center);
    j1->setLink(l1);

    Joint* j2 = new Joint(Eigen::Vector3f(0.0f,1.0f,0.0f));
    j2->setCurrRotation(0,0,1,60);
    //j2->setCurrRotation(0,0,45);
    j1->addChild(j2);
    Link *l2 = new Link(ldata1, ldata1center);
    j2->setLink(l2);

    Joint *j3 = new Joint(Eigen::Vector3f(0.0f,1.0f,0.0f));
    j3->setCurrRotation(0,0,1,10);

    gluPerspective(60, this->width()/float(this->height()), 0.00001,20000);
    gluLookAt(0,0,20,0,0,0,0,1,0);

    target << 0, 0, 0, 1;

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

void GLWidget::update()
{
    this->repaint();
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    Eigen::Vector4f targetP = this->target;

    drawCircle(targetP.head<3>(), 0.02, colorTarget);

    this->root->draw(Eigen::Matrix4f::Identity(), Eigen::Quaternionf::Identity());
    Kinematic::inverseKinematics(this->root->getChildren()[0]->getChildren()[0], this->target, 0.0005, 0.01);
//    std::cout << "root:\n" << this->root->getPosition() << "\nroot rot:\n" << this->root->getAcumRotation().toRotationMatrix() << "\nroot transf global:\n" << this->root->getTransformGlobal() << std::endl;
//    std::cout << "j1:\n" << this->root->getChildren()[0]->getPosition() << "\nJ1 rot:\n" << this->root->getChildren()[0]->getAcumRotation().toRotationMatrix() << "\nj1 transf global:\n" << this->root->getChildren()[0]->getTransformGlobal() << std::endl;
//    std::cout << "j2:\n" << this->root->getChildren()[0]->getChildren()[0]->getPosition() << "\nj2 rot:\n" << this->root->getChildren()[0]->getChildren()[0]->getAcumRotation().toRotationMatrix() << "\nj2 transf global:\n" << this->root->getChildren()[0]->getChildren()[0]->getTransformGlobal() << std::endl;
//    std::cout << "upwards from j2 " << this->root->getChildren()[0]->getChildren()[0]->numJointsHierarchyUpwards() << std::endl;
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    //Sejam as coordenadas da tela, vamos mapeá-las para coordenadas de display e então para coordenadas de mundo


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

