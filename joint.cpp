#include "joint.h"
#include <glm/gtc/matrix_transform.hpp>


std::vector<Joint *> Joint::getChildren() const
{
    return children;
}
Joint::Joint()
{

}

Joint::Joint(glm::vec3 offset)
{
    this->offset = offset;
}

Joint::Joint(glm::vec3 offset, float maxRotation, float minRotation)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
}

void Joint::setCurrRotation(float r)
{
    float rot;
    if (r > this->maxRotation) {
        rot = this->maxRotation;
    } else if (r < this->minRotation) {
        rot = this->minRotation;
    } else {
        rot = r;
    }
    this->currRotation = rot;
}

void Joint::setLink(Link *l)
{
    this->link = l;
}

void Joint::setParent(Joint *parent)
{
    this->parent = parent;
}

void Joint::addChild(Joint* child)
{
    child->setParent(this);
    this->children.push_back(child);
}

Joint *Joint::getNextChild()
{
    if (this->lastChildVisited < this->children.size()){
        return this->children.at(this->lastChildVisited);
    } else {
        this->lastChildVisited = 0;
        return NULL;
    }

}

/*
 * Recebe uma matriz de transformação vinda do link pai,
 * gera matrizes de transformação de acordo com os parâmetros da junta
 * (seu offset e seu ângulo atual), concatena e passa para o link
 * para desenho
 */
void Joint::draw(glm::mat4 transformation)
{
    glm::mat4 jointTrans = glm::translate(glm::mat4(1.0f), this->offset);
    glm::mat4 jointRot = glm::rotate(glm::mat4(1.0f), this->currRotation, glm::vec3(0,0,1));

    glm::mat4 concatTransform = transformation * jointTrans * jointRot;
    this->link->draw(concatTransform);

    for (const auto &childJoint : this->children) {
        childJoint->draw(concatTransform);
    }
}

