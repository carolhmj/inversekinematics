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
void Kinematic::applyPose(Joint *root, std::vector<float> pose)
{
    Joint *curr = NULL;
    std::queue<Joint*> jointList;
    jointList.push(root);
    for (const float &angle : pose){
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
std::vector<float> Kinematic::getPose(Joint *root)
{
    Joint *curr = NULL;
    std::vector<float> pose;
    std::queue<Joint*> jointList;
    jointList.push(root);
    while (jointList.size() > 0){
        curr = jointList.front();
        jointList.pop();
        pose.push_back(curr->getCurrRotation());
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
 * de uma junta específica a uma mudança no end effector. Inicialmente
 * vamos tratar de juntas em um plano bidimensional x-y, cuja rotação
 * é dada em termos do eixo de rotação (0,0,1)
 */
Eigen::MatrixXf Kinematic::jacobian(Joint *root, Eigen::Vector3f end)
{
    /* Linhas da Jacobiana. Dependem da quantidade de mudança que vai
     * ocorrer (se apenas linear ou se linear e rotacional)
     */
    int numRows = 2;
    /* Colunas da Jacobiana. Dependem da quantidade de juntas que temos
     */
    int numCols = root->numJointsHierarchy();

    //Criamos a Jacobiana agora!
    Eigen::MatrixXf jacobian(numRows,numCols);

    //Juntas num vetor para bem maior conveniência
    std::vector<Joint*> joints = root->flattenHierarchy();
    //Eixo de rotação
    Eigen::Vector3f rotationAxis(0.0,0.0,1.0);
    /* Colocando os termos na matrix. Sabemos que a mudança linear
     * é o produto vetorial do vetor da junta ao end effector
     */
    for (int j = 0; j < numCols; j++){
        //Vetor da junta ao end effector
        Eigen::Vector3f jointToEnd = end - joints.at(j)->getPosition().head<3>();
        Eigen::Vector3f cross = rotationAxis.cross(jointToEnd);
        for (int i = 0; i < numRows; i++){
            jacobian(i,j) = cross(i);
        }
    }

    return jacobian;
}

/*
 * Dado um end effector e o alvo desejado, calcula os ângulos das juntas para o end effector alcançar aquele alvo
 * Para isso calcula a jacobiana, a pseudoinversa, e resolve para encontrar os ângulos
 */
Eigen::Vector4f Kinematic::inverseKinematics(Joint *root, Eigen::Vector3f end, Eigen::Vector3f target, int timestep)
{
    cout << "target - end: " << target-end << "\n";
    //Vetor velocidade
    Eigen::Vector2f v = ((target - end)/timestep).head<2>();
    cout << "velocidade" << v << "\n";
    //Jacobiana
    Eigen::MatrixXf J = jacobian(root,end);
    cout << "jacobiana\n" << J << "\n";
    /*
     * Aplicando os passos mostrados na página 209 do livro
     */
    //Eigen::MatrixXf m = J * J.transpose();
    Eigen::Vector2f beta = (J * J.transpose()).householderQr().solve(v);
    cout << "beta\n" << beta << "\n";
    Eigen::VectorXf thetaVar = J.transpose() * beta;
    cout << "thetaVar:\n" << thetaVar << "\n";
    //Atualizando os ângulos
    std::vector<Joint*> joints = root->flattenHierarchy();
    int i = 0;
    for (auto &j : joints) {
        float updatedAngle = j->getCurrRotation() + thetaVar(i)*timestep;
        j->setCurrRotation(updatedAngle);
        i++;
    }
    flush(cout);
    return Eigen::Vector4f(0,0,0,0);
}

/*
 * Dado um end effector e o link contendo o end effector (ponto do meio), calcula os ângulos das juntas para o
 * end effector alcançar aquele alvo.
 * Para isso calcula a jacobiana, a pseudoinversa, e resolve para encontrar os ângulos
 */
void Kinematic::inverseKinematics(Joint *root, int linkEnd, Eigen::Vector3f target, int timestep)
{
    //Pegamos todas as juntas para encontrarmos o end effector
    std::vector<Joint*> joints = root->flattenHierarchy();
    if (linkEnd >= joints.size()) {
        return;
    }
    Eigen::Vector3f end = joints.at(linkEnd)->getLink()->getCenterPointTransformed();
    cout << "end" << end << "\n";
    //Vetor velocidade
    Eigen::Vector2f v = ((target - end)/timestep).head<2>();
    cout << "velocidade" << v << "\n";
    //Jacobiana
    Eigen::MatrixXf J = jacobian(root,end);
    cout << "jacobiana\n" << J << "\n";
    /*
     * Aplicando os passos mostrados na página 209 do livro
     */
    Eigen::Vector2f beta = (J * J.transpose()).householderQr().solve(v);
    cout << "beta\n" << beta << "\n";
    Eigen::VectorXf thetaVar = J.transpose() * beta;
    cout << "thetaVar:\n" << thetaVar << "\n";
    int i = 0;
    for (auto &j : joints) {
        float updatedAngle = j->getCurrRotation() + thetaVar(i)*timestep;
        j->setCurrRotation(updatedAngle);
        i++;
    }
    flush(cout);
}

