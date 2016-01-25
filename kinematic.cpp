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
Eigen::MatrixXf Kinematic::jacobian(Joint *start, Eigen::Vector4f endEff)
{
    /* Linhas da Jacobiana. Dependem do tipo de mudança que vai
     * ocorrer (se apenas linear ou se linear e rotacional). Teremos
     * 3 linhas para mudança apenas linear
     */
    int numRows = 3;
    /* Colunas da Jacobiana. Dependem da quantidade de juntas que temos
     */
    int numCols = 3*start->numJointsHierarchyUpwards();

    //Criamos a Jacobiana agora!
    Eigen::MatrixXf jacobian(numRows,numCols);

    //Matriz m, com as rotações
    Eigen::Matrix3f m;
    //Matriz px
    Eigen::Matrix3f px;

    Eigen::Vector4f p;

    Joint *effector = start;

    int blockstart = 0;
    while (effector != NULL){
        p = endEff - effector->getPosition();
        px <<   0  , p(2), -p(1),
              -p(2),  0  ,  p(0),
               p(1),-p(0),   0  ;

        m = (effector->getTransformGlobal().block<3,3>(0,0)).transpose();
        jacobian.block<3,3>(0,3*blockstart) = px*m;
        blockstart++;
        effector = effector->getParent();
    }

    return jacobian;
}

Eigen::MatrixXf Kinematic::pseudoInverse(Eigen::MatrixXf M)
{
    Eigen::MatrixXf M1(M.rows(), M.rows());
    M1 = M * M.transpose();
    Eigen::MatrixXf M2(M1.rows(), M1.cols());
    M2 = M1.inverse();
    Eigen::MatrixXf M3(M.cols(), M.rows());
    M3 = M.transpose() * M2;
    return M3;
}

/*
 * Dado um end effector e a posição do end effector, calcula os ângulos das juntas para o
 * end effector alcançar aquele alvo.
 * Para isso calcula a jacobiana, a pseudoinversa, e resolve para encontrar os ângulos
 */
void Kinematic::inverseKinematics(Joint *effector, Eigen::Vector4f target, float adjustFactor, float tolerance)
{
    Eigen::Vector4f effectorPosition = effector->getPosition();
    std::cout << "calculando nova aproximação, diferença de " << (effectorPosition - target).norm() << std::endl;
    if ((effectorPosition - target).norm() < tolerance) {
        std::cout << "tolerancia atingida\n";
        return;
    }
    flush(std::cout);
    Eigen::Vector3f e = (adjustFactor * (effectorPosition-target)).head<3>();

    Eigen::MatrixXf jacobianM = jacobian(effector, target);
    Eigen::MatrixXf jacobianPseudoInverse = pseudoInverse(jacobianM);
    Eigen::MatrixXf orientations = jacobianPseudoInverse * e;

    orientations = (180.f/M_PI) * orientations;
    Joint *currEffector = effector;
    int jointStart = 0;
    while (currEffector != NULL){
        currEffector->acumCurrRotation(orientations(3*jointStart, 0), orientations(3*jointStart+1, 0), orientations(3*jointStart+2, 0));
        currEffector = currEffector->getParent();
        jointStart++;
    }

    flush(cout);
}

