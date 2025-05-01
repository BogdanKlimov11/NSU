QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    light_window.cpp \
    mesh_widget.cpp \
    object_loader.cpp

HEADERS += \
    mesh_widget.h \
    light_window.h \
    object_loader.h

RESOURCES += \
    icons.qrc \
    objects.qrc \
    shaders.qrc \
    textures.qrc

DISTFILES +=
