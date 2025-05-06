#pragma once

#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <memory>

class EarthMaterial {
public:
    EarthMaterial();
    void bind(QOpenGLShaderProgram* program, bool useTexture, bool useNormalMap, bool useNightTexture);

private:
    std::shared_ptr<QOpenGLTexture> dayTexture;
    std::shared_ptr<QOpenGLTexture> nightTexture;
    std::shared_ptr<QOpenGLTexture> normalMap;
};
