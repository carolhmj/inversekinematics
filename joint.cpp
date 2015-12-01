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

string Joint::getName() const
{
    return name;
}

void Joint::setName(const string &value)
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

Joint *Joint::getParent() const
{
    return parent;
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

Joint::Joint(Eigen::Vector3f offset, Eigen::Vector4f rotationAxis, string name)
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

    //std::cout << "concatTransform in joint " << this->name << "\n" << concatTransform << "\n";

    Eigen::Vector4f pos(0.0,0.0,0.0,1.0);
    this->position = concatTransform * pos;
    //std::cout << "position in joint " << this->name << "\n" << this->position << "\n";
    this->rotationAxisTransform = concatTransform * this->rotationAxis;
    this->rotationAxisTransform.normalize();
//    std::cout << "rotationAxis in joint " << this->name << "\n" << this->rotationAxisTransform << "\n";
//    flush(std::cout);
//    std::cout << "xxxxxxxxx\n";
//    flush(std::cout);
    //Desenha a junta como um círculo
    if (DRAWJOINTS) {
        glColor3f(1,0,0);
        //Desenha um círculo que representa a junta
        glBegin(GL_LINE_LOOP);
            for (int i=0; i < 360; i++) {
               float degInRad = DEG2RAD(i);
               glVertex2f(this->position[0] + cos(degInRad)*JOINTRADIUS, this->position[1] + sin(degInRad)*JOINTRADIUS);
            }
        glEnd();
        //Desenha as juntas
        if (this->name[0] == 'r') {
            glColor3f(1,0,0);
        } else if (this->name[0] == 'j'){
            glColor3f(0,1,0);
        } else if (this->name[0] == 'f'){
            glColor3f(0,0,1);
        } else if (this->name[0] == 'v'){
            glColor3f(0,1,1);
        }

            glBegin(GL_LINES);
                glVertex3f(position(0), position(1), position(2));
                glVertex3f(position(0) + rotationAxisTransform(0), position(1) + rotationAxisTransform(1), position(2) + rotationAxisTransform(2));
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
