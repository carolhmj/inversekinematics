#include "glwidget.h"
#include <QTimerEvent>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <QDebug>
#include <iostream>
#include <math.h>
#include <GL/gl.h>
#include <QObject>
#include <Qt>

using namespace std;

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    connect(&timer,SIGNAL(timeout()),this,SLOT(update()));
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0,0,0,1);

    std::vector<Eigen::Vector4f> ldata1;
    ldata1.push_back(Eigen::Vector4f(0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,3,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,3,0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,3,0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,3,-0.5,1));

    ldata1.push_back(Eigen::Vector4f(-0.5,1,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,-0.5,1));
    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));

    ldata1.push_back(Eigen::Vector4f(0.5,1,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,0.5,1));
    ldata1.push_back(Eigen::Vector4f(-0.5,1,-0.5,1));

    Eigen::Vector4f lcenter(0,2,0,1);
    Eigen::Vector3f offZero(0,0,0);
    Eigen::Vector3f off(0,3,0);
    Eigen::Vector4f xAxis = Eigen::Vector4f::UnitX(), yAxis = Eigen::Vector4f::UnitY(), zAxis = Eigen::Vector4f::UnitZ();

    this->root = new Joint(offZero,xAxis);
    this->root->setName("root (rx)");
    Joint* ry = new Joint(offZero,yAxis);
    ry->setName("ry");
    ry->setCurrRotation(45);
    this->root->addChild(ry);

    Joint* rz = new Joint(offZero,zAxis);
    rz->setName("rz");
    ry->addChild(rz);
    Link *l = new Link(ldata1,lcenter);
    rz->setLink(l);

    Joint* jx = new Joint(off, xAxis);
    jx->setName("jx");
    rz->addChild(jx);

    Joint* jy = new Joint(offZero,yAxis);
    jy->setName("jy");
    jy->setCurrRotation(45);
    jx->addChild(jy);

    Joint* jz = new Joint(offZero,zAxis);
    jz->setName("jz");
    jy->addChild(jz);

    Link *l1 = new Link(ldata1,lcenter);
    jz->setLink(l1);
    jz->setCurrRotation(90);

    Joint *fx = new Joint(off, xAxis);
    fx->setName("fx");
    jz->addChild(fx);

    Joint *fy = new Joint(offZero, yAxis);
    fy->setName("fy");
    fx->addChild(fy);

    Joint *fz = new Joint(offZero, zAxis);
    fz->setName("fz");
    fy->addChild(fz);

    Link *l2 = new Link(ldata1,lcenter);
    fz->setLink(l2);
    fz->setCurrRotation(90);

    Joint *vx = new Joint(off, xAxis);
    vx->setName("vx");
    fz->addChild(vx);

    Joint *vy = new Joint(offZero, yAxis);
    vy->setName("vy");
    vx->addChild(vy);

    Joint *vz = new Joint(offZero, zAxis);
    vz->setName("vz");
    vy->addChild(vz);

    Link *l3 = new Link(ldata1,lcenter);
    vz->setLink(l3);

//    std::vector<float> pose;
//    pose.push_back(0.0f);
//    pose.push_back(90.0f);
//    pose.push_back(30.0f);
//    Kinematic::applyPose(this->root, pose);

    projection = Projections::ortho(-10,10,-10,10,-10,10);
    view = Projections::lookAt(Eigen::Vector3f(0.0f,0.0f,1.0f),Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,1.0f,0.0f));
    target << 0, 0, 0, 1;
    end << 0, 0, 0, 1;

    timer.start(1.6666);
    //static const QMetaMethod valueChangedSignal = QMetaMethod::fromSignal(&this->timer::timeout);
    qDebug() << "timer is active? " << timer.isActive() << "\n";
    //qDebug() << "is signal connected? " << timer->isSignalConnected(valueChangedSignal);
    Eigen::Vector3f x = Eigen::Vector3f::UnitX();
    Eigen::Vector4f y;
    y.head<3>() = x;
    y(3) = 1;
    cout << y << "\n";
    y = Eigen::Vector4f::UnitY();
    cout << y << "\n";
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
    qDebug() << "update called\n";
    this->repaint();
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    Eigen::Vector4f endP = /*projection * view **/ this->end;
    Eigen::Vector4f targetP = /*projection * view **/ this->target;

    drawCircle(endP.head<3>(), 0.02, colorEnd);
    drawCircle(targetP.head<3>(), 0.02, colorTarget);
    //Kinematic::inverseKinematics(this->root, 11, target, Eigen::Vector3f(0,0,0), 100);
    this->root->draw(projection * view);
    //this->root->draw(Eigen::Matrix4f::Identity());
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    //Sejam as coordenadas da tela, vamos mapeá-las para coordenadas de display e então para coordenadas de mundo
    int w = this->width(), h = this->height();
    qDebug() << "w: " << w << " h: " << h << "\n";
    Eigen::Vector4f screenCoord(event->x(), event->y(), 0, 1);
    cout << "mouse coord: " << screenCoord << "\n";

    Eigen::Vector4f displayCoord;
    displayCoord[0] = ((screenCoord[0] - (w/2.0)) /* * 5.0*/) / (w/2.0);
    displayCoord[1] = (-(screenCoord[1] - (h/2.0)) /* * 5.0*/) / (h/2.0);
    displayCoord[2] = 0.0;
    displayCoord[3] = 1;
    cout << "display coord: " << displayCoord << "\n";

    Eigen::Matrix4f displayToWorld = (projection * view).inverse();
    Eigen::Vector4f worldCoord = displayToWorld * displayCoord;
    cout << "world coord: " << worldCoord << "\n";
    flush(cout);

    if (event->button() == Qt::LeftButton){
        this->end = displayCoord;
        int i = 0;
        for (auto j : this->root->flattenHierarchy()){
            cout << "joint " << i << " " << j->getName() << "\n";
            i++;
        }

    } else if (event->button() == Qt::RightButton){
        this->target = displayCoord;
        //this->target = worldCoord;
    } else if (event->button() == Qt::MidButton){
        Kinematic::inverseKinematics(this->root, 11, target, Eigen::Vector3f(0,180,0), 1);
//        for (auto &c : this->root->flattenHierarchy()){
//            cout << "\n=======\njoint " << c->getName().toStdString();
//            cout << "\ntransformed joint point\n" << c->getPosition();
//            cout << "\ntransformed rotationaxis\n" << c->getRotationAxisTransform();
//        }
        flush(cout);
    }


//    //Matriz de mapeamento das coordenadas do display para as coordenadas da tela
//    Eigen::Matrix4f displayToScreen;
//    displayToScreen << w/2+1.0/w,   0  , 0 , w/2,
//                        0  , -h/2+1.0/h , 0 , h/2,
//                        0  ,   0   , 1 ,  0 ,
//                        0  ,   0   , 0 ,  1 ;

//    Eigen::Matrix4f screenToDisplay = displayToScreen.inverse();
//    Eigen::Vector4f displayCoord = screenToDisplay * screenCoord;
//    cout << "display coord: " << displayCoord << "\n";

//    //Mapeamento das coordenadas do display para o mundo é o inverso da projeção
//    Eigen::Matrix4f displayToWorld = (projection * view).inverse();
//    Eigen::Vector4f worldCoord = displayToWorld * displayCoord;
//    cout << "world coord: " << worldCoord << "\n";
//    flush(cout);
//    if (event->button() == Qt::LeftButton){
//        this->end = displayCoord.head<3>();
//        //this->end = worldCoord.head<3>();
//    } else if (event->button() == Qt::RightButton){
//        this->target = displayCoord.head<3>();
//        //this->target = worldCoord.head<3>();
//    }
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

