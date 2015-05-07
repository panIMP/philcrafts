TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp


INLUDEPATH


include(deployment.pri)
qtcAddDeployment()

