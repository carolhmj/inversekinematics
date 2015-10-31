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
public:
    Link();
    Link(std::vector<Eigen::Vector4f> data, Eigen::Vector3f centerPoint);
    void draw(Eigen::Matrix4f transform);
};

#endif // LINK_H
