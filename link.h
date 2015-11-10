#ifndef LINK_H
#define LINK_H

#include <eigen3/Eigen/Dense>
#include <vector>

/*
 * Classe que representa o link de uma figura articulada
 */
class Link
{
private:
    //Vértices do link
    std::vector<Eigen::Vector4f> data;
    //Ponto central dos dados
    Eigen::Vector3f centerPoint;
    //Ponto central dos dados de acordo com a transformação deles
    Eigen::Vector3f centerPointTransformed;
public:
    Link();
    Link(std::vector<Eigen::Vector4f> data, Eigen::Vector3f centerPoint);
    void draw(Eigen::Matrix4f transform);
    Eigen::Vector3f getCenterPoint() const;
    void setCenterPoint(const Eigen::Vector3f &value);
    Eigen::Vector3f getCenterPointTransformed() const;
    void setCenterPointTransformed(const Eigen::Vector3f &value);
};

#endif // LINK_H
