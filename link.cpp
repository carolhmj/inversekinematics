#include "link.h"
#include <GL/gl.h>
#include <iostream>
#include <QDebug>
#include "drawing.h"
using namespace std;

Eigen::Vector3f Link::getCenterPoint() const
{
    return centerPoint;
}

void Link::setCenterPoint(const Eigen::Vector3f &value)
{
    centerPoint = value;
}

Eigen::Vector3f Link::getCenterPointTransformed() const
{
    return centerPointTransformed;
}

void Link::setCenterPointTransformed(const Eigen::Vector3f &value)
{
    centerPointTransformed = value;
}
Link::Link()
{

}

Link::Link(std::vector<Eigen::Vector4f> data, Eigen::Vector3f centerPoint)
{
    this->data = data;
    this->centerPoint = centerPoint;
}

Link::Link(std::vector<Eigen::Vector4f> data, std::vector<int> drawOrder, Eigen::Vector3f centerPoint)
{
    this->data = data;
    this->centerPoint = centerPoint;
    this->drawOrder = drawOrder;
}

void Link::draw(Eigen::Matrix4f transform)
{
    Eigen::Vector4f centerPMod(centerPoint[0],centerPoint[1],centerPoint[2],1);
    this->centerPointTransformed = (transform * centerPMod).head<3>();
    //cout << "center point trans:\n" << this->centerPointTransformed << "\n";
    glColor3f(0,1,0);
    glBegin(GL_LINE_LOOP);
        for (int i=0; i < 360; i++) {
           float degInRad = DEG2RAD(i);
           glVertex2f(this->centerPointTransformed[0] + cos(degInRad)*CENTERPOINTRADIUS, this->centerPointTransformed[1] + sin(degInRad)*CENTERPOINTRADIUS);
        }
    glEnd();
    //Drawing::drawSphere(centerPointTransformed(0), centerPointTransformed(1), centerPointTransformed(2), 0.0000000001, 50, 50);
    glColor3f(1,1,1);

    glColor3f(1,0,1);

    if (this->drawOrder.empty()) {
        //qDebug() << "empty draw order\n";
        glBegin(GL_TRIANGLE_FAN);
        for (const auto &v : this->data){
            auto vTrans = transform * v;
            glVertex3f(vTrans(0), vTrans(1), vTrans(2));
        }
        glEnd();
    } else {
        //qDebug() << "XXXXXXXXXXXXXXXX\n";
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < this->drawOrder.size(); i = i + 3){
            //qDebug() << "not empty draw order\n";
            //qDebug() << "i: " << drawOrder[i] << " i+1: " << drawOrder[i+1] << " i+2: " << drawOrder[i+2] << "\n";
            Eigen::Vector4f v1 = transform * data[drawOrder[i]], v2 = transform * data[drawOrder[i+1]], v3 = data[drawOrder[i+2]];
            glVertex3f(v1[0], v1[1], v1[2]);
            glVertex3f(v2[0], v2[1], v2[2]);
            glVertex3f(v3[0], v3[1], v3[2]);
        }
        glEnd();
    }

    glColor3f(1,1,1);

}

