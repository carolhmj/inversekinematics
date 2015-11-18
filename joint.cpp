#include "joint.h"
#include <QDebug>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <queue>
#include <iostream>
#include "GL/gl.h"
#include <QString>

using namespace std;
std::vector<Joint *> Joint::getChildren() const
{
    return children;
}

float Joint::getCurrRotation() const
{
    return currRotation;
}

Link *Joint::getLink() const
{
    return link;
}

QString Joint::getName() const
{
    return name;
}

void Joint::setName(const QString &value)
{
    name = value;
}

Eigen::Vector4f Joint::getRotationAxis() const
{
    return rotationAxis;
}

Eigen::Vector4f Joint::getRotationAxisTransform() const
{
    return rotationAxisTransform;
}
Joint::Joint()
{

}

Joint::Joint(Eigen::Vector3f offset)
{
    this->offset = offset;
    this->rotationAxis = Eigen::Vector4f::UnitZ();
    this->rotationAxisTransform = rotationAxis;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

Joint::Joint(Eigen::Vector3f offset, float maxRotation, float minRotation)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->rotationAxis = Eigen::Vector4f::UnitZ();
    this->rotationAxisTransform = rotationAxis;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

Joint::Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->rotationAxis = rotationAxis;
    this->rotationAxis.normalize();
    this->rotationAxisTransform = this->rotationAxis;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

Joint::Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis, QString name)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->rotationAxis = rotationAxis;
    this->rotationAxis.normalize();
    this->rotationAxisTransform = this->rotationAxis;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
    this->name = name;
}

Joint::Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis, float maxRotation, float minRotation)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->rotationAxis = rotationAxis;
    this->rotationAxis.normalize();
    this->rotationAxisTransform = this->rotationAxis;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
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

std::vector<Joint*> Joint::flattenHierarchy()
{
    Joint *curr = NULL;
    std::vector<Joint*> pose;
    std::queue<Joint*> jointList;
    jointList.push(this);
    while (jointList.size() > 0){
        curr = jointList.front();
        jointList.pop();
        pose.push_back(curr);
        std::vector<Joint*> children = curr->getChildren();
        if (children.size() > 0){
            for (auto &c : children) {
                jointList.push(c);
            }
        }
    }
    return pose;
}

int Joint::numJointsHierarchy()
{

    if (children.empty()){  //Está numa folha
        return 1;
    } else { //Percorre a hierarquia e vê quantos filhos existem
        int acum = 0;
        for (auto c : this->children){
            acum += c->numJointsHierarchy();
        }
        return acum + 1; //Adiciona ele próprio
    }

}

/*
 * Recebe uma matriz de transformação vinda do link pai,
 * gera matrizes de transformação de acordo com os parâmetros da junta
 * (seu offset e seu ângulo atual), concatena e passa para o link
 * para desenho
 */
void Joint::draw(Eigen::Matrix4f transformation)
{

    Eigen::Affine3f trans(Eigen::Translation3f(this->offset));
    Eigen::Matrix4f jointTrans = trans.matrix();
    //qDebug() << "rotation angle" << DEG2RAD(this->currRotation) << "\n";
    Eigen::Affine3f rotation(Eigen::AngleAxisf(DEG2RAD(this->currRotation),this->rotationAxis.head<3>()));
    Eigen::Matrix4f jointRot = rotation.matrix();

    Eigen::Matrix4f concatTransform = transformation * jointTrans * jointRot;
    //Eigen::Matrix4f concatTransform = transformation * jointRot * jointTrans;
    this->rotationAxisTransform = concatTransform * this->rotationAxis;

    Eigen::Vector4f pos(0.0,0.0,0.0,1.0);
//    if (this->parent != NULL) {
//        //qDebug() << "non-null parent\n";
//        pos = this->parent->getPosition();
//    }
    this->position = concatTransform * pos;
    //qDebug() << "position: " << this->position[0] << " " << this->position[1] << " " << this->position[2] << "\n";
    //Desenha a junta como um círculo
    if (DRAWJOINTS) {
        glColor3f(1,0,0);
        if (this->name == QString("rz")){
            glColor3f(0,1,1);
        }
        glBegin(GL_LINE_LOOP);
            for (int i=0; i < 360; i++) {
               float degInRad = DEG2RAD(i);
               glVertex2f(this->position[0] + cos(degInRad)*JOINTRADIUS, this->position[1] + sin(degInRad)*JOINTRADIUS);
            }
        glEnd();
        glColor3f(1,1,1);
    }

    if (this->link != NULL) {
        this->link->draw(concatTransform);
    }
    for (const auto &childJoint : this->children) {
        childJoint->draw(concatTransform);
    }
}


Eigen::Vector4f Joint::getPosition() const
{
    return position;
}
