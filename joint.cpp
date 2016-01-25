#include "joint.h"
#include <QDebug>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <queue>
#include <GL/gl.h>

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

void Joint::drawPyramid(Eigen::Vector4f bottom, Eigen::Vector4f top, float base, Eigen::Matrix4f mv)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(mv.data());

    float s = base/2.f;
    const GLfloat pyramidVertices[] = {
        top(0), top(1), top(2),
        bottom(0)+s, bottom(1), bottom(2)+s,
        bottom(0)+s, bottom(1), bottom(2)-s,
        bottom(0)-s, bottom(1), bottom(2)-s,
        bottom(0)-s, bottom(1), bottom(2)+s,
    };

    const GLfloat pyramidColors[] = {
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
    };

    GLuint index[] = {
        0,2,1,
        0,3,2,
        0,4,3,
        0,1,4,
        3,1,4,
        1,3,2
    };

    glPolygonMode(GL_FRONT, GL_FILL);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColorPointer(3, GL_FLOAT, 0, pyramidColors);
    glVertexPointer(3, GL_FLOAT, 0, pyramidVertices);
    glDrawElements(GL_TRIANGLES, 18, GL_UNSIGNED_INT, index);
}


Eigen::Matrix4f Joint::getTransform() const
{
    return transform;
}

Eigen::Matrix4f Joint::getTransformGlobal() const
{
    return transformGlobal;
}

Eigen::Vector4f Joint::getPositionGlobal() const
{
    return positionGlobal;
}

Joint *Joint::getParent() const
{
    return parent;
}
Joint::Joint()
{

}

Joint::Joint(Eigen::Vector4f offset)
{
    this->position = offset;
    this->positionGlobal = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}

Joint::Joint(Eigen::Vector4f offset, Eigen::Vector3f color)
{
    this->position = offset;
    this->color = color;
}

Joint::Joint(Eigen::Vector4f offset, Eigen::Vector3f maxRotation, Eigen::Vector3f minRotation)
{
    this->position = offset;
    this->maxRotation = maxRotation;
    this->minRotation = minRotation;
    this->positionGlobal = Eigen::Vector4f(0.0,0.0,0.0,1.0);
}


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
    //Transforma os ângulos em ângulo-eixo
    Eigen::AngleAxisf xRotation(DEG2RAD(x), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(DEG2RAD(y), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(DEG2RAD(z), Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf rotationIncrement(xRotation * yRotation * zRotation);
    rotationIncrement.normalize();
    currRotation = currRotation * rotationIncrement;
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
 * Atualiza a matriz de transformação atual da junta de acordo com a matriz pai, também atualiza a posição e orientação
 * da junta
 */
void Joint::updateTransform()
{

    //Calcula a transformação
    Eigen::Affine3f trans(Eigen::Translation3f(this->position.head<3>()));
    Eigen::Matrix4f jointTrans = trans.matrix();
    Eigen::Affine3f rotation(this->currRotation);
    Eigen::Matrix4f jointRot = rotation.matrix();


    this->transform = jointRot * jointTrans;
//    this->transform = jointTrans * jointRot;

    //Atualiza a transformação global da junta a partir da transformação global do pai
    if (this->parent != NULL) {
        this->transformGlobal = this->parent->getTransformGlobal() * this->transform;
//        this->transformGlobal = this->transform * this->parent->getTransformGlobal();
        this->acumRotation = this->parent->getAcumRotation() * this->currRotation;
        this->positionGlobal = this->parent->getTransformGlobal() * this->getPosition();
        std::cout << "position global inside if:\n" << this->positionGlobal << std::endl;
        std::cout << "mult:\n" << this->parent->getTransformGlobal().transpose() * this->getPosition() << std::endl;
        std::cout << "parent transform global:\n" << this->parent->getTransformGlobal() << std::endl;
        std::cout << "this position:\n" << this->position << std::endl;

    } else {
        this->transformGlobal = this->transform;
        this->acumRotation = this->currRotation;
        this->positionGlobal = this->position;
    }

    std::cout << "position global outside if:\n" << this->positionGlobal << std::endl;
//    std::cout << "get transform\n" << this->transform << std::endl;
//    std::cout << "get transform global\n" << this->transformGlobal << std::endl;

    this->positionGlobal;
    for (const auto &childJoint : this->children) {
        childJoint->updateTransform();
    }
}

/*
 * Recebe uma matriz de model-view/projection e desenha a junta
 */
void Joint::draw(Eigen::Matrix4f transformation)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();
    glLoadMatrixf(transformation.data());
//    glMultMatrixf(transformGlobal.data());

    //Eigen::Vector3f e = acumRotation.toRotationMatrix() * Eigen::Vector3f(0.f,1.f,0.f);
//    glColor3f(1,1,1);
//    glBegin(GL_LINES);
//        //glVertex3f(positionGlobal(0), positionGlobal(1), positionGlobal(2));
//        //glVertex3f(positionGlobal(0)+e(0), positionGlobal(1)+e(1), positionGlobal(2)+e(2));
//        glVertex3f(position(0), position(1), position(2));
//        glVertex3f(position(0), position(1)+1.f, position(2));
//    glEnd();

    Eigen::Vector4f v(0.,1.,0.,0.);
    //v << e,1;l
    //std::cout << "position:\n" << transformGlobal*position << "position+v:\n" << transformGlobal*(position+v) << std::endl;
    if (this->children.size() > 0){
        drawPyramid(positionGlobal, this->children[0]->getPositionGlobal(), 1, transformation);
    }
    glPopMatrix();
    for (const auto &childJoint : this->children) {
        //std::cout << "called child joint\n";
        childJoint->draw(transformation);
    }

}


Eigen::Vector4f Joint::getPosition() const
{
    return positionGlobal;
}
