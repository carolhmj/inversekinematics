#include "link.h"
#include <GL/gl.h>
#include <iostream>
using namespace std;
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
}

