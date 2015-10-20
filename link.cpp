#include "link.h"
#include <GL/gl.h>

Link::Link()
{

}

Link::Link(std::vector<glm::vec4> data, glm::vec3 centerPoint)
{
    this->data = data;
    this->centerPoint = centerPoint;
}

void Link::draw(glm::mat4 transform)
{
    glBegin(GL_TRIANGLE_FAN);
    for (const auto &v : this->data){
        auto vTrans = transform * v;
        glVertex3f(vTrans.x, vTrans.y, vTrans.z);
    }
    glEnd();
}

