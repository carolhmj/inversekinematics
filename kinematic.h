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
    static Eigen::MatrixXf jacobian(Joint *start, Eigen::Vector4f target);
    static Eigen::MatrixXf pseudoInverseJacobian(Eigen::MatrixXf M);
    static void inverseKinematics(Joint *effector, Eigen::Vector4f target, float adjustFactor, float tolerance);
    //Função para calcular a pseudo-inversa, fonte: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
    template<typename _Matrix_Type_>
    static _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
};

#endif // KINEMATIC_H
