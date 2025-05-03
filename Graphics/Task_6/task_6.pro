QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    earth_geometry.cpp \
    earth_material.cpp \
    main.cpp \
    main_window.cpp

HEADERS += \
    earth_geometry.h \
    earth_material.h \
    main_window.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

DISTFILES +=
