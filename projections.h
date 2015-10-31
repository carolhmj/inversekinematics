#ifndef PROJECTIONS_H
#define PROJECTIONS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

/*
 * Classe de utilidade para funções de projeção, já que o Eigen não
 * tem elas :/
 */

class Projections
{
public:
    Projections();
    static Eigen::Matrix4f lookAt(Eigen::Vector3f eye, Eigen::Vector3f center, Eigen::Vector3f up);
    static Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float near, float far);
};

#endif // PROJECTIONS_H
