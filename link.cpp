#include "link.h"
#include <GL/gl.h>
#include <iostream>
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

void Link::draw(Eigen::Matrix4f transform)
{
    glBegin(GL_TRIANGLE_FAN);
    for (const auto &v : this->data){
        auto vTrans = transform * v;
        glVertex3f(vTrans(0), vTrans(1), vTrans(2));
    }
    glEnd();

    glPointSize(5);
    glColor3f(0,1,0);
    glBegin(GL_POINTS);
    Eigen::Vector4f centerPMod(centerPoint[0],centerPoint[1],centerPoint[2],1);
    centerPointTransformed = (transform * centerPMod).head<3>();
    //cout << "center point transformed: " << centerPointTransformed << "\n";
    glVertex3f(centerPointTransformed(0),centerPointTransformed(1),centerPointTransformed(2));
    glEnd();
    //glPointSize(1);
    //glColor3f(1,1,1);
}

