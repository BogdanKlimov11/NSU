#pragma once

#include <QOpenGLTexture> // Для работы с текстурами OpenGL
#include <QOpenGLShaderProgram> // Для работы с шейдерными программами OpenGL
#include <memory> // Для использования умных указателей (std::shared_ptr)

// Определение класса MaterialEarth
class MaterialEarth {
public:
    // Конструктор класса
    MaterialEarth();
    // Метод для привязки текстур к шейдерной программе
    void bind(QOpenGLShaderProgram* program);

private:
    // Умный указатель на текстуру дневной поверхности Земли
    std::shared_ptr<QOpenGLTexture> day_texture_;
    // Умный указатель на карту нормалей для освещения
    std::shared_ptr<QOpenGLTexture> normal_map_;
};
