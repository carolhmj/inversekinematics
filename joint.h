#ifndef JOINT_H
#define JOINT_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <link.h>
#include <vector>
#include <cfloat>
#include <cmath>

#define DEG2RAD(x) float(x * (M_PI / 180.0))
#define DRAWJOINTS true
#define JOINTRADIUS 0.01
/*
 * Classe que representa uma junta de uma figura articulada. Trabalhando
 * agora com juntas ball cuja orientação é dada por um quaternion
 */
class Joint
{
private:
    //Posição da junta em coordenadas do mundo. Varia dependendo da transformação passada para a junta
    Eigen::Vector4f position;
    //Offset da junta em relação ao pai
    Eigen::Vector3f offset;
    //Rotação da junta, em euler e quaternion
    Eigen::Vector3f currRotationEuler;
    Eigen::Quaternionf currRotation;
    //Eixo de rotação
    Eigen::Vector3f rotationAxis;
    //Quaternion acumulado com o pai
    Eigen::Quaternionf acumRotation;
    //Limites de rotação
    Eigen::Vector3f maxRotation = Eigen::Vector3f(FLT_MAX,FLT_MAX,FLT_MAX), minRotation = Eigen::Vector3f(FLT_MIN,FLT_MIN,FLT_MIN);
    //Link associado
    Link* link = NULL;
    //Junta pai na hierarquia
    Joint* parent = NULL;
    //Juntas filhas na hierarquia
    std::vector<Joint*> children;
    //Último filho visitado, usado na função getNextChild
    unsigned int lastChildVisited = 0;
public:
    Joint();
    Joint(Eigen::Vector3f offset);
    Joint(Eigen::Vector3f offset, Eigen::Vector3f maxRotation, Eigen::Vector3f minRotation);
    void setCurrRotation(Eigen::Vector3f r);
    void setCurrRotation(float x, float y, float z);
    void acumCurrRotation(float x, float y, float z);
    Eigen::Vector3f getCurrRotationEuler() const;
    void setLink(Link* l);
    void setParent(Joint* parent);
    void addChild(Joint *child);
    std::vector<Joint*> getChildren() const;
    Joint* getNextChild();
    //Retorna toda a hierarquia a partir dessa junta como uma lista de juntas
    std::vector<Joint*> flattenHierarchy();
    //Retorna o número de juntas na hierarquia, INCLUSIVE
    int numJointsHierarchy();
    //Retorna o número de juntas superiores na hierarquia, INCLUSIVE
    int numJointsHierarchyUpwards();
    void draw(Eigen::Matrix4f transformation, Eigen::Quaternionf quaternion);
    Eigen::Vector4f getPosition() const;
    Link *getLink() const;
    Eigen::Quaternionf getCurrRotation() const;
    Eigen::Vector3f getRotationAxis() const;
    Eigen::Quaternionf getAcumRotation() const;
};

#endif // JOINT_H
