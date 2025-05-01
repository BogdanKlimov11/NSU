QT += core gui opengl openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

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

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    icons.qrc \
    objects.qrc \
    shaders.qrc \
    textures.qrc

# Добавь эту строку:
LIBS += -lopengl32

DISTFILES +=
