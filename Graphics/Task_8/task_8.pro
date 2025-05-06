QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    road_material.cpp \
    road_widget.cpp

HEADERS += \
    road_material.h \
    road_widget.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

LIBS += \
    -lopengl32
