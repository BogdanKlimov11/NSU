#pragma once

#include <QtOpenGL>
#include <vector>
#include <memory>

enum TextrureType {
    ROCK,
    ROAD,
    CRACKS
};

enum FilteringType {
    NEAREST,
    LINEAR,
    LINEAR_MIP_MAP_LINEAR,
};

class Road final : public QOpenGLFunctions {
public:
    Road();
    void initialize(QOpenGLShaderProgram* prog);
    void render();
    void initUniforms();
    void setTotalMatrix(const QMatrix4x4& newTotalMatrix);
    void changeFilterType(TextrureType texture, FilteringType filt);

private:
    void updateTextures();
    void setUniforms() const;
    void initTextures();
    void update();
    QOpenGLShaderProgram* program = nullptr;
    const std::array<QOpenGLTexture::Filter, 3> possibleFilters = {
        QOpenGLTexture::Nearest, QOpenGLTexture::Linear, QOpenGLTexture::LinearMipMapLinear
    };
    std::vector<std::shared_ptr<QOpenGLTexture>> textures;
    std::vector<std::string> textureNames;
    std::vector<QOpenGLTexture::Filter> textureFiltering;
    QMatrix4x4 totalMatrix;
    std::vector<QVector3D> vertices = {{}, {}, {}, {}};
    std::vector<GLuint> indices = {0, 1, 2, 3};
    const QVector3D normal = {0, 1, 0};
    GLint posAttr = -1;
    GLint normalUniform = -1;
    GLint totalMatrixUniform = -1;
    GLint widhtUniform = -1;
    GLint heightUniform = -1;
    float widht = 1.0f;
    float height = 100.0f;
};
