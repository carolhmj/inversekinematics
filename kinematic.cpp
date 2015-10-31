#include "kinematic.h"
#include <queue>

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
    int numRows = 3;
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
        Eigen::Vector3f jointToEnd;
        //Precisamos primeiro encontrar a posição da junta em coordenadas globais, e para isso, precisamos da transformação dela!
        //jointToEnd = end - Eigen::Vector3f();
        Eigen::Vector3f cross = rotationAxis.cross(jointToEnd);
        for (int i = 0; i < numRows; i++){
            jacobian(i,j) = cross(i);
        }
    }

    return jacobian;
}

