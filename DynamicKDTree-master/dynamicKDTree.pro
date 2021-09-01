TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS_RELEASE += -O3

SOURCES += main.cpp \
    sKdTree.cpp \
    dKdTree.cpp

HEADERS += \
    addon.h \
    sKdTree.h \
    dKdTree.h \
    lazyrebuild.h
