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

