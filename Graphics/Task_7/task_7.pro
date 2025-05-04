QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    mesh_widget.cpp \
    object_loader.cpp \
    light_window.cpp

HEADERS += \
    mesh_widget.h \
    object_loader.h \
    light_window.h

RESOURCES += \
    icons.qrc \
    objects.qrc \
    shaders.qrc \
    textures.qrc
