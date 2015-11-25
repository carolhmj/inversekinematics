#include "link.h"
#include <GL/gl.h>
#include <iostream>
using namespace std;

Eigen::Vector4f Link::getCenterPoint() const
{
    return centerPoint;
}

void Link::setCenterPoint(const Eigen::Vector4f &value)
{
    centerPoint = value;
}

Eigen::Vector4f Link::getCenterPointTransformed() const
{
    return centerPointTransformed;
}

void Link::setCenterPointTransformed(const Eigen::Vector4f &value)
{
    centerPointTransformed = value;
}
Link::Link()
{

}

Link::Link(std::vector<Eigen::Vector4f> data, Eigen::Vector4f centerPoint)
{
    this->data = data;
    this->centerPoint = centerPoint;
}

void Link::draw(Eigen::Matrix4f transform)
{

    this->centerPointTransformed = transform * centerPoint;
    glColor3f(0,1,0);
    glBegin(GL_LINE_LOOP);
        for (int i=0; i < 360; i++) {
           float degInRad = DEG2RAD(i);
           glVertex2f(this->centerPointTransformed[0] + cos(degInRad)*CENTERPOINTRADIUS, this->centerPointTransformed[1] + sin(degInRad)*CENTERPOINTRADIUS);
        }
    glEnd();
    glColor3f(1,1,1);

    glBegin(GL_TRIANGLES);
    for (const auto &v : this->data){
        auto vTrans = transform * v;
        glVertex3f(vTrans(0), vTrans(1), vTrans(2));
    }
    glEnd();

}

