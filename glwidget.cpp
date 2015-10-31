#include "glwidget.h"
#include <QTimer>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <QDebug>
#include <iostream>

using namespace std;

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    timer = new QTimer(this);
    qDebug() << "connecttest: " << QObject::connect(timer,SIGNAL(timeout()),this,SLOT(update())) << "\n";
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0,0,0,1);
    this->root = new Joint(Eigen::Vector3f(-2.0f,0.0f,0.0f));
    std::vector<Eigen::Vector4f> ldata1;
    ldata1.push_back(Eigen::Vector4f(0.5,1,0,1));
    ldata1.push_back(Eigen::Vector4f(0.5,-1,0,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,-1,0,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,0,1));
    Link *l1 = new Link(ldata1,Eigen::Vector3f(0.0,0.0,0.0));
    this->root->setLink(l1);
    //this->root->setCurrRotation(0.0f);
    Joint* j1 = new Joint(Eigen::Vector3f(0.0f,2.5f,0.0f));
    this->root->addChild(j1);
    Link *l2 = new Link(ldata1,Eigen::Vector3f(0.0,0.0,0.0));
    j1->setLink(l2);
    //j1->setCurrRotation(45.0f);

    std::vector<float> pose;
    pose.push_back(0.0f);
    pose.push_back(45.0f);
    Kinematic::applyPose(this->root, pose);

    this->rotAngle = 0.0f;
    timer->start(60);
    cout << timer->isActive() << "\n";
    qDebug() << "num joints hierarchy: " << this->root->numJointsHierarchy() << "\n";
    flush(cout);
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
    cout << "rotAngle: " << this->rotAngle << "\n";
    flush(cout);
    this->rotAngle += 0.0012f;
    if (this->rotAngle >= 360.0f){
        this->rotAngle = 0.0f;
    }
    this->root->setCurrRotation(this->rotAngle);
    this->repaint();
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    float halfW = float(this->width())/2.0;
//    float halfH = float(this->height())/2.0;
    Eigen::Matrix4f perspMatrix = Projections::ortho(-5,5,-5,5,-5,5);
    Eigen::Matrix4f view = Projections::lookAt(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,1.0f,0.0f));
    this->root->draw(perspMatrix * view);
}

