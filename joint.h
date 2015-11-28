#ifndef JOINT_H
#define JOINT_H

#include <eigen3/Eigen/Dense>
#include <link.h>
#include <vector>
#include <cfloat>
#include <cmath>
#include <string>

#define DEG2RAD(x) float(x * (M_PI / 180.0))
#define DRAWJOINTS true
#define JOINTRADIUS 0.01
/*
 * Classe que representa uma junta de uma figura articulada.
 * Vamos tentar mudar a representação de orientação
 */
class Joint
{
private:
    //Posição da junta em coordenadas do mundo. Varia dependendo da transformação passada para a junta
    Eigen::Vector4f position;
    //Offset da junta em relação ao pai
    Eigen::Vector3f offset;
    //Rotação da junta
    float currRotation = 0.0;
    //Eixo de rotação da junta, em coordenadas locais
    Eigen::Vector4f rotationAxis;
    //Eixo de rotação da junta, em coordenadas do mundo
    Eigen::Vector4f rotationAxisTransform;
    //Limites de rotação
    float maxRotation = FLT_MAX, minRotation = FLT_MIN;
    //Link associado
    Link* link = NULL;
    //Junta pai na hierarquia
    Joint* parent = NULL;
    //Juntas filhas na hierarquia
    std::vector<Joint*> children;
    //Último filho visitado, usado na função getNextChild
    unsigned int lastChildVisited = 0;
    //Nome da junta
    std::string name;
public:
    Joint();
    Joint(Eigen::Vector3f offset);
    Joint(Eigen::Vector3f offset, float maxRotation, float minRotation);
    Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis);
    Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis, std::string name);
    Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis, float maxRotation, float minRotation);
    void setCurrRotation(float r);
    float getCurrRotation() const;
    void setLink(Link* l);
    void setParent(Joint* parent);
    void addChild(Joint *child);
    std::vector<Joint*> getChildren() const;
    Joint* getNextChild();
    //Retorna toda a hierarquia a partir dessa junta como uma lista de juntas
    std::vector<Joint*> flattenHierarchy();
    //Retorna o número de juntas na hierarquia
    int numJointsHierarchy();
    void draw(Eigen::Matrix4f transformation);
    //void draw(Eigen::Matrix4f view, Eigen::Matrix4f model);
    Eigen::Vector4f getPosition() const;
    Link *getLink() const;
    std::string getName() const;
    void setName(const std::string &value);
    Eigen::Vector4f getRotationAxis() const;
    Eigen::Vector4f getRotationAxisTransform() const;
    Joint *getParent() const;
};

#endif // JOINT_H
