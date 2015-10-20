#include "glwidget.h"
#include <QTimer>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
    this->root = new Joint(glm::vec3(-2.0f,0.0f,0.0f));
    std::vector<glm::vec4> ldata1;
//    ldata1.push_back(glm::vec4(.5,.5,0,1));
//    ldata1.push_back(glm::vec4(1,0,0,1));
//    ldata1.push_back(glm::vec4(-1,0,0,1));
//    Link *l1 = new Link(ldata1,glm::vec3(0.5,0.25,0));
    ldata1.push_back(glm::vec4(0.5,1,0,1));
    ldata1.push_back(glm::vec4(0.5,-1,0,1));
    ldata1.push_back(glm::vec4(-0.5,-1,0,1));
    ldata1.push_back(glm::vec4(-0.5,1,0,1));
    Link *l1 = new Link(ldata1,glm::vec3(0.0f));
    this->root->setLink(l1);
    //this->root->setCurrRotation(0.0f);
    Joint* j1 = new Joint(glm::vec3(0.0f,2.5f,0.0f));
    this->root->addChild(j1);
    Link *l2 = new Link(ldata1,glm::vec3(0.0f));
    j1->setLink(l2);
    //j1->setCurrRotation(45.0f);

    std::vector<float> pose;
    pose.push_back(0.0f);
    pose.push_back(90.0f);
    Kinematic::applyPose(this->root, pose);

    this->rotAngle = 0.0f;
    timer->start(60);
    cout << timer->isActive() << "\n";
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
    glm::mat4 perspMatrix = glm::ortho(5.0f,-5.0f,-5.0f,5.0f,1.0f,-1.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0.0f,0.0f,-1.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f));
//    int i = 0;
//    for (auto c : Kinematic::getPose(this->root)){
//        qDebug() << "angle " << i++ <<": " << c << "\n";
//    }
    this->root->draw(perspMatrix * view);
}

