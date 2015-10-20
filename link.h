#ifndef LINK_H
#define LINK_H

#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <vector>

/*
 * Classe que representa o link de uma figura articulada
 */
class Link
{
private:
    //Vértices do link
    std::vector<glm::vec4> data;
    //Ponto central dos dados
    glm::vec3 centerPoint;
public:
    Link();
    Link(std::vector<glm::vec4> data, glm::vec3 centerPoint);
    void draw(glm::mat4 transform);
};

#endif // LINK_H
