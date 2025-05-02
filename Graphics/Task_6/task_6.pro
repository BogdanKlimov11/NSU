QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    earth_material.cpp \
    main.cpp \
    main_window.cpp \
    square_window.cpp

HEADERS += \
    earth_material.h \
    main_window.h \
    square_window.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

DISTFILES +=
