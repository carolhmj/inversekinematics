#ifndef LINK_H
#define LINK_H

#include <eigen3/Eigen/Dense>
#include <vector>

#define DEG2RAD(x) float(x * (M_PI / 180.0))
#define CENTERPOINTRADIUS 0.01

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
    //Ordem de desenho dos vértices
    std::vector<int> drawOrder;
public:
    Link();
    Link(std::vector<Eigen::Vector4f> data, Eigen::Vector3f centerPoint);
    Link(std::vector<Eigen::Vector4f> data, std::vector<int> drawOrder, Eigen::Vector3f centerPoint);
    void draw(Eigen::Matrix4f transform);
    Eigen::Vector3f getCenterPoint() const;
    void setCenterPoint(const Eigen::Vector3f &value);
    Eigen::Vector3f getCenterPointTransformed() const;
    void setCenterPointTransformed(const Eigen::Vector3f &value);
};

#endif // LINK_H
