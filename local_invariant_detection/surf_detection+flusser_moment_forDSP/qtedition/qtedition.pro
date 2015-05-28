TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

INCLUDEPATH += D:/opencv2.4.10/opencv/build/include
LIBS += D:\opencv2.4.10\opencv\build\x64\vc12\lib\opencv_highgui2410.lib
LIBS += D:/opencv2.4.10/opencv/build/x64/vc12/lib/opencv_imgproc2410.lib


SOURCES += \
    ../src/algBase.cpp \
    ../src/algFunction.cpp \
    ../src/algManage.cpp \
    ../src/algPreproc.cpp \
    ../src/main.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../inc/algBase.h \
    ../inc/algFunction.h \
    ../inc/algManage.h \
    ../inc/algParam.h \
    ../inc/algPreproc.h

