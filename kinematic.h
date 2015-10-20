#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <vector>
#include <joint.h>
/*
 * Classe com funções para cinemática direta e inversa
 */
class Kinematic
{
public:
    Kinematic();
    static void applyPose(Joint *root, std::vector<float> pose);
};

#endif // KINEMATIC_H
