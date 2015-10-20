#ifndef JOINT_H
#define JOINT_H

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <link.h>
#include <vector>
#include <cfloat>

/*
 * Classe que representa uma junta de uma figura articulada. Por enquanto vou
 * trabalhar apenas com juntas rotacionais 1D. 
 */
class Joint
{
private:
    //Offset da junta em relação ao pai
    glm::vec3 offset;
    //Rotação da junta
    float currRotation;
    //Limites de rotação
    float maxRotation = FLT_MAX, minRotation = FLT_MIN;
    //Link associado
    Link* link;
    //Junta pai na hierarquia
    Joint* parent;
    //Juntas filhas na hierarquia
    std::vector<Joint*> children;
public:
    Joint();
    Joint(glm::vec3 offset);
    Joint(glm::vec3 offset, float maxRotation, float minRotation);
    void setCurrRotation(float r);
    void setLink(Link* l);
    void setParent(Joint* parent);
    void addChild(Joint *child);
    void draw(glm::mat4 transformation);
};

#endif // JOINT_H
