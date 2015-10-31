#include "projections.h"

Projections::Projections()
{

}

Eigen::Matrix4f Projections::lookAt(Eigen::Vector3f eye, Eigen::Vector3f center, Eigen::Vector3f up)
{
    Eigen::Vector3f f = center - eye;
    f.normalize();
    Eigen::Vector3f u(up);
    u.normalize();
    Eigen::Vector3f s(f.cross(u));
    s.normalize();
    u = s.cross(f);

    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
    ret(0,0) = s(0);
    ret(1,0) = s(1);
    ret(2,0) = s(2);
    ret(0,1) = u(0);
    ret(1,1) = u(1);
    ret(2,1) = u(2);
    ret(0,2) = -f(0);
    ret(1,2) = -f(1);
    ret(2,2) = -f(2);
    ret(3,0) = -s.dot(eye);
    ret(3,1) = -u.dot(eye);
    ret(3,2) = f.dot(eye);
    //return Eigen::Matrix4f();
    return ret;
}

Eigen::Matrix4f Projections::ortho(float left, float right, float bottom, float top, float near, float far)
{
    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
    ret(0,0) = 2.0/(right-left);
    ret(1,1) = 2.0/(top-bottom);
    ret(2,2) = -2.0/(far-near);
    ret(0,3) = -(right+left)/(right-left);
    ret(1,3) = -(top+bottom)/(top-bottom);
    ret(2,3) = -(far+near)/(far-near);
    //return Eigen::Matrix4f();
    return ret;
}

