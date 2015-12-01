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
    static Eigen::MatrixXf jacobian(Joint *root, Eigen::Vector4f endPos);
    static void inverseKinematics(Joint *root, int linkEnd, Eigen::Vector4f targetPos, Eigen::AngleAxisf targetRot, int timestep);
};

#endif // KINEMATIC_H
