#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTimer>
#include "joint.h"
#include "link.h"
#include "kinematic.h"
#include "projections.h"

class GLWidget : public QGLWidget
{
protected:
    Joint* root;
    float rotAngle;
    QTimer *timer;
public:
    explicit GLWidget(QWidget *parent = 0);
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
public slots:
    void update();
};

#endif // GLWIDGET_H
