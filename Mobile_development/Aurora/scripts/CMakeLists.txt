cmake_minimum_required(VERSION 3.12)

project(CheckersGame)

set(CMAKE_CXX_STANDARD 17)

find_package(Qt6 REQUIRED COMPONENTS Quick)

add_executable(CheckersGame
    main.cpp
    GameLogic.cpp
    Checkers.qml
    CheckersGame.qml
)

qt_add_qml_module(CheckersGame
    URI CheckersGame
    VERSION 1.0
    QML_FILES Checkers.qml CheckersGame.qml
)

target_link_libraries(CheckersGame
    Qt6::Quick
)
