#include "kinematic.h"
#include <queue>
#include <iostream>

using namespace std;
Kinematic::Kinematic()
{

}

/*
 * Aplica uma pose a uma estrutura começando da junta root; a ordem dos
 * elementos na pose é a ordem de uma busca em largura nos nós
 */
void Kinematic::applyPose(Joint *root, std::vector<Eigen::Vector3f> pose)
{
    Joint *curr = NULL;
    std::queue<Joint*> jointList;
    jointList.push(root);
    for (const auto &angle : pose){
        curr = jointList.front();
        jointList.pop();
        curr->setCurrRotation(angle);
        std::vector<Joint*> children = curr->getChildren();
        if (children.size() > 0){
            for (auto c : children){
                jointList.push(c);
            }
        }
    }
}

/*
 * Percorre a estrutura como uma busca em largura e retorna os ângulos atuais dos
 * links
 */
std::vector<Eigen::Vector3f> Kinematic::getPose(Joint *root)
{
    Joint *curr = NULL;
    std::vector<Eigen::Vector3f> pose;
    std::queue<Joint*> jointList;
    jointList.push(root);
    while (jointList.size() > 0){
        curr = jointList.front();
        jointList.pop();
        pose.push_back(curr->getCurrRotationEuler());
        std::vector<Joint*> children = curr->getChildren();
        if (children.size() > 0){
            for (auto c : children){
                jointList.push(c);
            }
        }
    }
    return pose;
}

/*
 * Retorna a matriz Jacobiana, em que cada termo relaciona a mudança
 * de uma junta específica a uma mudança no end effector. Agora vamos
 * tratar de juntas rotacionais com 3 DOFs (ball joints)
 */
Eigen::MatrixXf Kinematic::jacobian(Joint *root, Eigen::Vector3f end)
{
    /* Linhas da Jacobiana. Dependem do tipo de mudança que vai
     * ocorrer (se apenas linear ou se linear e rotacional). Como temos
     * mudança linear e rotacional de um end effector, temos 6 linhas
     */
    int numRows = 6;
    /* Colunas da Jacobiana. Dependem da quantidade de juntas que temos
     */
    int numCols = root->numJointsHierarchy();

    //Criamos a Jacobiana agora!
    Eigen::MatrixXf jacobian(numRows,numCols);

    //Juntas num vetor para bem maior conveniência
    std::vector<Joint*> joints = root->flattenHierarchy();
    //Eixo de rotação
    Eigen::Vector3f rotationAxis;
    /* Colocando os termos na matrix. Sabemos que a mudança linear
     * é o produto vetorial do vetor da junta ao end effector com
     * o eixo de rotação. Já a mudança rotacional é o eixo de rotação
     */
    for (int j = 0; j < numCols; j++){
        //Vetor da junta ao end effector
        Eigen::Vector3f jointToEnd = end - joints.at(j)->getPosition().head<3>();
        //Eixo de rotação
        rotationAxis = Eigen::AngleAxisf(root->getAcumRotation()).axis();
        cout << "rotation axis:\n" << rotationAxis << "\n";
        Eigen::Vector3f cross = rotationAxis.cross(jointToEnd);
        //Mudança linear
        for (int i = 0; i < 3; i++){
            jacobian(i,j) = cross(i);
        }
        //Mudança rotacional
        for (int i = 3; i < 6; i++){
            jacobian(i,j) = rotationAxis(3-i);
        }
    }

    return jacobian;
}

/*
 * Dado um end effector e o link contendo o end effector (ponto do meio), calcula os ângulos das juntas para o
 * end effector alcançar aquele alvo.
 * Para isso calcula a jacobiana, a pseudoinversa, e resolve para encontrar os ângulos
 */
void Kinematic::inverseKinematics(Joint *root, int linkEnd, Eigen::Vector3f targetPos, Eigen::Quaternionf targetRot, int timestep)
{
    //Pegamos todas as juntas para encontrarmos o end effector
    std::vector<Joint*> joints = root->flattenHierarchy();
    for (auto &j : joints) {
        //cout << j->getCurrRotation() << "\n";
    }
    if (linkEnd >= joints.size()) {
        return;
    }

    Eigen::Vector3f endPos = joints.at(linkEnd)->getLink()->getCenterPointTransformed();
    Eigen::Quaternionf endRot = joints.at(linkEnd)->getCurrRotation();

    Eigen::Matrix<Eigen::Quaternionf, 6, 1> v;

    //Vetor velocidade
    Eigen::Vector3f x = endPos - targetPos;
    for (int i = 0; i < 3; i++) {
        v(i) = Eigen::Quaternionf(x(i), 0, 0, 0);
    }
    //If you want to find a quaternion diff such that diff * q1 == q2, then you need to use the multiplicative inverse:
    Eigen::Quaternionf diff = targetRot * endRot.inverse();
    diff.normalize();
    for (int i = 3; i < 6; i++) {
        v(i) = diff[3-i];
    }

//    cout << "velocidade" << v << "\n";
    //Jacobiana
    Eigen::MatrixXf J = jacobian(root,endPos);
//    cout << "jacobiana\n" << J << "\n";

    /*
     * Como eu não sei se é possível fazer o Eigen trabalhar com matrizes de quaternions,
     * vou usar o método da jacobiana transposta
     */
    Eigen::MatrixXf JT = J.transpose();
    Eigen::Matrix<Eigen::Quaternionf, 6, 1> theta;
    for (int i = 0; i < JT.rows(); i++){
        Eigen::Quaternionf sum = Eigen::Quaternionf::Identity();
        for (int j = 0; j < JT.cols(); j++){
            Eigen::Quaternionf q(JT(i,j) * v(j).coeffs());
            q.normalize();
            sum = sum.coeffs() + q.coeffs();
        }
        theta(i) = sum;
    }

    //Atualizando...
    int i = 0;
    for (auto &j : joints) {
        Eigen::Quaternionf updatedRotation = Eigen::Quaternionf::Identity();
        updatedRotation = j->getCurrRotation().coeffs() + theta(i).coeffs()*timestep;
        updatedRotation.normalize();
        //j->setCurrRotation(updatedRotation);
    }

    /*
     * Aplicando os passos mostrados na página 209 do livro
     */
//    Eigen::Matrix<Eigen::Quaternionf, 6, 1> beta = (J * J.transpose()).householderQr().solve(v);
//    cout << "beta\n" << beta << "\n";
//    Eigen::Matrix<Eigen::Quaternionf, 6, 1> thetaVar = J.transpose() * beta;
//    cout << "thetaVar:\n" << thetaVar << "\n";
//    int i = 0;
////    for (auto &j : joints) {
////        float updatedAngle = j->getCurrRotation() + thetaVar(i)*timestep;
////        j->setCurrRotation(updatedAngle);
////        i++;
////    }
//    flush(cout);
}

