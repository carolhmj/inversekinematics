#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <vector>
#include <joint.h>
#include <eigen3/Eigen/Dense>
/*
 * Classe com funções para cinemática direta e inversa
 */
class Kinematic
{
public:
    Kinematic();
    static void applyPose(Joint *root, std::vector<float> pose);
    static std::vector<float> getPose(Joint *root);
    static std::vector<Joint*> flattenHierarchy();
    Eigen::MatrixXf jacobian(Joint *root, Eigen::Vector3f end);
};

#endif // KINEMATIC_H
