QT += core gui opengl openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    main_window.cpp \
    material_earth.cpp \
    square_window.cpp

HEADERS += \
    main_window.h \
    material_earth.h \
    square_window.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    shaders.qrc \
    textures.qrc

DISTFILES +=
