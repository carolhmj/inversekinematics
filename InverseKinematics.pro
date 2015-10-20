#-------------------------------------------------
#
# Project created by QtCreator 2015-10-17T14:18:43
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = InverseKinematics
TEMPLATE = app
CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    joint.cpp \
    link.cpp \
    glwidget.cpp

HEADERS  += mainwindow.h \
    joint.h \
    link.h \
    glwidget.h

FORMS    += mainwindow.ui

INCLUDEPATH += $$PWD/glm/
