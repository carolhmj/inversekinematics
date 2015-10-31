#ifndef JOINT_H
#define JOINT_H

#include <eigen3/Eigen/Dense>
#include <link.h>
#include <vector>
#include <cfloat>
#include <cmath>

#define DEG2RAD(x) float(x * (M_PI / 180.0))

/*
 * Classe que representa uma junta de uma figura articulada. Por enquanto vou
 * trabalhar apenas com juntas rotacionais 1D. 
 */
class Joint
{
private:
    //Offset da junta em relação ao pai
    Eigen::Vector3f offset;
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
    //Último filho visitado, usado na função getNextChild
    unsigned int lastChildVisited = 0;
public:
    Joint();
    Joint(Eigen::Vector3f offset);
    Joint(Eigen::Vector3f offset, float maxRotation, float minRotation);
    void setCurrRotation(float r);
    float getCurrRotation() const;
    void setLink(Link* l);
    void setParent(Joint* parent);
    void addChild(Joint *child);
    std::vector<Joint*> getChildren() const;
    Joint* getNextChild();
    //Retorna o número de juntas na hierarquia
    int numJointsHierarchy();
    void draw(Eigen::Matrix4f transformation);
};

#endif // JOINT_H
