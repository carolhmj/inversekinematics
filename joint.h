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
    Eigen::Vector4f positionGlobal = Eigen::Vector4f::Zero();
    //Offset da junta em relação ao pai
    Eigen::Vector4f position = Eigen::Vector4f::Zero();
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
    //Matriz de transformação da junta
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //Matriz de transformação global da junta
    Eigen::Matrix4f transformGlobal = Eigen::Matrix4f::Identity();
    //Cor da junta
    Eigen::Vector3f color = Eigen::Vector3f(1.,1.,1.);
public:
    Joint();
    Joint(Eigen::Vector4f position);
    Joint(Eigen::Vector4f position, Eigen::Vector3f color);
    Joint(Eigen::Vector4f position, Eigen::Vector3f maxRotation, Eigen::Vector3f minRotation);

    void setCurrRotation(float x, float y, float z);
    void acumCurrRotation(float x, float y, float z);

    //Retorna toda a hierarquia a partir dessa junta como uma lista de juntas
    std::vector<Joint*> flattenHierarchy();
    //Retorna o número de juntas na hierarquia
    int numJointsHierarchy();
    //Número de juntas na hierarquia, contando para cima
    int numJointsHierarchyUpwards();
    //Atualiza as transformações da junta de acordo com os seus parâmetros e a transformação da junta pai
    void updateTransform();
    //Desenha a junta, podendo receber uma transformação model-view
    void draw(Eigen::Matrix4f transformation);


    /*
     * Getters e setters variados
     */
    void setCurrRotation(Eigen::Vector3f r);
    Eigen::Vector4f getPosition() const;
    Link *getLink() const;
    Eigen::Quaternionf getCurrRotation() const;
    Eigen::Quaternionf getAccumRotation();
    Eigen::Vector3f getRotationAxis() const;
    Eigen::Quaternionf getAcumRotation() const;
    void drawPyramid(Eigen::Vector4f bottom, Eigen::Vector4f top, float base, Eigen::Matrix4f mv = Eigen::Matrix4f::Identity());
    Eigen::Matrix4f getTransform() const;
    Eigen::Matrix4f getTransformGlobal() const;
    Eigen::Vector4f getPositionGlobal() const;
    Joint *getParent() const;
    void setLink(Link* l);
    void setParent(Joint* parent);
    void addChild(Joint *child);
    std::vector<Joint*> getChildren() const;
    Joint* getNextChild();
    Eigen::Vector3f getCurrRotationEuler() const;

};

#endif // JOINT_H
