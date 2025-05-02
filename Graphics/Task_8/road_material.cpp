#include "road_material.h"

Road::Road() {
    textureNames = {"Rock", "Road", "Cracks"};
    textureFiltering = {QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear};
}

void Road::changeFilterType(const TextrureType texture, const FilteringType filt) {
    textureFiltering[texture] = possibleFilters[filt];
    if (textures[texture]) {
        textures[texture]->setMinMagFilters(possibleFilters[filt], possibleFilters[filt]);
        if (possibleFilters[filt] == QOpenGLTexture::LinearMipMapLinear) {
            textures[texture]->generateMipMaps();
        }
    }
}

void Road::updateTextures() {
    for (size_t i = 0; i < textures.size(); ++i) {
        if (textures[i]) {
            glActiveTexture(GL_TEXTURE0 + i);
            textures[i]->bind();
            program->setUniformValue(textureNames[i].c_str(), static_cast<int>(i));
        }
    }
}

void Road::initTextures() {
    QImage rockImage(":/road_2.jpg");
    rockImage.isNull();
    textures.emplace_back(std::make_shared<QOpenGLTexture>(rockImage.mirrored()));
    textures[ROCK]->setWrapMode(QOpenGLTexture::Repeat);
    textures[ROCK]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[ROCK]->generateMipMaps();

    QImage roadImage(":/road_1.jpg");
    roadImage.isNull();
    textures.emplace_back(std::make_shared<QOpenGLTexture>(roadImage.mirrored()));
    textures[ROAD]->setWrapMode(QOpenGLTexture::Repeat);
    textures[ROAD]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[ROAD]->generateMipMaps();

    QImage cracksImage(":/road_3.jpg");
    cracksImage.isNull();
    textures.emplace_back(std::make_shared<QOpenGLTexture>(cracksImage.mirrored()));
    textures[CRACKS]->setWrapMode(QOpenGLTexture::Repeat);
    textures[CRACKS]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[CRACKS]->generateMipMaps();
}

void Road::setTotalMatrix(const QMatrix4x4& newTotalMatrix) {
    totalMatrix = newTotalMatrix;
}

void Road::initialize(QOpenGLShaderProgram* prog) {
    initializeOpenGLFunctions();
    program = prog;
    initUniforms();
    initTextures();
    update();
}

void Road::render() {
    program->bind();
    setUniforms();
    updateTextures();
    glVertexAttribPointer(posAttr, 3, GL_FLOAT, GL_FALSE, 0, vertices.data());
    glEnableVertexAttribArray(posAttr);
    glDrawElements(GL_TRIANGLE_STRIP, indices.size(), GL_UNSIGNED_INT, indices.data());
    glDisableVertexAttribArray(posAttr);
    program->release();
}

void Road::initUniforms() {
    posAttr = program->attributeLocation("posAttr");
    normalUniform = program->uniformLocation("normal");
    totalMatrixUniform = program->uniformLocation("totalMatrix");
    widhtUniform = program->uniformLocation("xSize");
    heightUniform = program->uniformLocation("zSize");
}

void Road::setUniforms() const {
    program->setUniformValue(normalUniform, normal);
    program->setUniformValue(totalMatrixUniform, totalMatrix);
    program->setUniformValue(widhtUniform, widht);
    program->setUniformValue(heightUniform, height);
}

void Road::update() {
    const float halfWidth = widht / 2.0f;
    vertices[0] = QVector3D(-halfWidth, 0.0f, 0.0f);
    vertices[1] = QVector3D(halfWidth, 0.0f, 0.0f);
    vertices[2] = QVector3D(-halfWidth, 0.0f, height);
    vertices[3] = QVector3D(halfWidth, 0.0f, height);
}
