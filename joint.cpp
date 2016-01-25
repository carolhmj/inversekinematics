#include "joint.h"
#include <QDebug>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <queue>
#include "GL/gl.h"

std::vector<Joint *> Joint::getChildren() const
{
    return children;
}

Link *Joint::getLink() const
{
    return link;
}

Eigen::Quaternionf Joint::getCurrRotation() const
{
    return currRotation;
}


Eigen::Quaternionf Joint::getAcumRotation() const
{
    return acumRotation;
}

Eigen::Matrix4f Joint::getTransformGlobal() const
{
    return transformGlobal;
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
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

Joint::Joint(Eigen::Vector3f offset, Eigen::Vector3f maxRotation, Eigen::Vector3f minRotation)
{
    this->offset = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->position = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

//void Joint::setCurrRotation(Eigen::Quaternionf r)
//{
//    this->currRotation = r;
//}

void Joint::setCurrRotation(Eigen::Vector3f r)
{
    if (r[0] > this->minRotation[0] && r[0] < this->maxRotation[0]){
        if (r[1] > this->minRotation[1] && r[1] < this->maxRotation[1]){
            if (r[2] > this->minRotation[2] && r[2] < this->maxRotation[2]){
                this->currRotationEuler = r;
                setCurrRotation(r[0],r[1],r[2]);
            }
        }
    }

}

void Joint::setCurrRotation(float x, float y, float z)
{
    this->currRotationEuler = Eigen::Vector3f(x,y,z);

    //Transforma os ângulos em ângulo-eixo
    Eigen::AngleAxisf xRotation(DEG2RAD(x), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(DEG2RAD(y), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(DEG2RAD(z), Eigen::Vector3f::UnitZ());

    //Concatena e normaliza para formar o quaternion
    this->currRotation = xRotation * yRotation * zRotation;
    this->currRotation.normalize();

    //Guarda o eixo de rotação
    this->rotationAxis = Eigen::AngleAxisf(currRotation).axis();
}

void Joint::acumCurrRotation(float x, float y, float z)
{
    this->currRotationEuler = this->currRotationEuler + Eigen::Vector3f(x,y,z);

    //Transforma os ângulos em ângulo-eixo
    Eigen::AngleAxisf xRotation(DEG2RAD(x), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(DEG2RAD(y), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(DEG2RAD(z), Eigen::Vector3f::UnitZ());

    //Concatena e normaliza para formar o quaternion
    Eigen::Quaternionf rotationAmount = xRotation * yRotation * zRotation;
    rotationAmount.normalize();
    this->currRotation = this->currRotation * rotationAmount;
    this->currRotation.normalize();

    //Guarda o eixo de rotação
    this->rotationAxis = Eigen::AngleAxisf(currRotation).axis();
}

Eigen::Vector3f Joint::getCurrRotationEuler() const
{
    return this->currRotationEuler;
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


int Joint::numJointsHierarchyUpwards()
{
    if (parent == NULL) { //É a raiz
        return 1;
    } else { //Percorre a hierarquia e vê quantos pais existem
        return parent->numJointsHierarchyUpwards() + 1;
    }
}

/*
 * Recebe uma matriz de transformação vinda do link pai,
 * gera matrizes de transformação de acordo com os parâmetros da junta
 * (seu offset e seu ângulo atual), concatena e passa para o link
 * para desenho
 */
void Joint::draw(Eigen::Matrix4f transformation, Eigen::Quaternionf quaternion)
{

    Eigen::Affine3f trans(Eigen::Translation3f(this->offset));
    Eigen::Matrix4f jointTrans = trans.matrix();
    //qDebug() << "rotation angle" << DEG2RAD(this->currRotation) << "\n";
    Eigen::Affine3f rotation(this->currRotation);
    Eigen::Matrix4f jointRot = rotation.matrix();

    Eigen::Matrix4f concatTransform = transformation * jointTrans * jointRot;

    Eigen::Vector4f pos(0.0,0.0,0.0,1.0);
    this->acumRotation = quaternion * this->currRotation;
    this->acumRotation.normalize();
    this->position = concatTransform * pos;
    this->transformGlobal = concatTransform;


    //qDebug() << "position: " << this->position[0] << " " << this->position[1] << " " << this->position[2] << "\n";
    //Desenha a junta como um círculo
    if (DRAWJOINTS) {
        glColor3f(1,0,0);
        glBegin(GL_LINE_LOOP);
            for (int i=0; i < 360; i++) {
               float degInRad = DEG2RAD(i);
               glVertex2f(this->position[0] + cos(degInRad)*JOINTRADIUS, this->position[1] + sin(degInRad)*JOINTRADIUS);
            }
        glEnd();
        glColor3f(1,1,1);
    }

    this->link->draw(concatTransform);

    for (const auto &childJoint : this->children) {
        childJoint->draw(concatTransform, acumRotation);
    }
}


Eigen::Vector4f Joint::getPosition() const
{
    return position;
}
