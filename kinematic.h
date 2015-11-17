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
    static void applyPose(Joint *root, std::vector<Eigen::Vector3f> pose);
    static std::vector<Eigen::Vector3f> getPose(Joint *root);
    static Eigen::MatrixXf jacobian(Joint *root, Eigen::Vector3f endEff);
    static void inverseKinematics(Joint *root, int linkEnd, Eigen::Vector3f targetPos, Eigen::Quaternionf targetRot, int timestep);
};

#endif // KINEMATIC_H
