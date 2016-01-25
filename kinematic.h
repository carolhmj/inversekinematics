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
    static Eigen::MatrixXf jacobian(Joint *start, Eigen::Vector4f endEff);
    static Eigen::MatrixXf pseudoInverse(Eigen::MatrixXf M);
    static void inverseKinematics(Joint *effector, Eigen::Vector4f target, float adjustFactor, float tolerance);
};

#endif // KINEMATIC_H
