#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTimer>
#include <QMouseEvent>
#include <eigen3/Eigen/Dense>
#include "joint.h"
#include "link.h"
#include "kinematic.h"
#include "projections.h"

class GLWidget : public QGLWidget
{
protected:
    Joint* root;
    QTimer timer;
    Eigen::Vector4f target;
    Eigen::Vector4f end;

    //Matrizes de projeção
    Eigen::Matrix4f projection;
    Eigen::Matrix4f view;

    //Cores
    float colorEnd[3] = {1,1,0};
    float colorTarget[3] = {1,0,1};

public:
    explicit GLWidget(QWidget *parent = 0);
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void drawCircle(Eigen::Vector3f center, float radius, float color[]);
public slots:
    void update();
};

#endif // GLWIDGET_H
