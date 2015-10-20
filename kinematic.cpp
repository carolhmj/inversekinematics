#include "kinematic.h"
#include <queue>

Kinematic::Kinematic()
{

}

/*
 * Aplica uma pose a uma estrutura começando da junta root; a ordem dos
 * elementos na pose é a ordem de uma busca em largura nos nós
 */
static void Kinematic::applyPose(Joint *root, std::vector<float> pose)
{
    Joint *curr;
    std::queue<Joint*> jointList;
    jointList.push(root);
    for (const float &angle : pose){
        curr = jointList.pop();
        curr->setCurrRotation(angle);
        std::vector children = curr->getChildren();
        if (children.size() > 0){
            for (auto c : children){
                jointList.push(c);
            }
        }
    }
}

