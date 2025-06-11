QT += core gui opengl openglwidgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    game_logic.cpp \
    object_generator.cpp \
    scene_settings.cpp

HEADERS += \
    billiard_widget.h

RESOURCES += \
    resources.qrc

LIBS += \
    -lopengl32
