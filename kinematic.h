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
    static Eigen::MatrixXf jacobian(Joint *root, Eigen::Vector3f endEff);
    static Eigen::Vector4f inverseKinematics(Joint *root, Eigen::Vector3f end, Eigen::Vector3f target, int timestep);
    static void inverseKinematics(Joint *root, int linkEnd, Eigen::Vector3f target, int timestep);
};

#endif // KINEMATIC_H
