#include "road_material.h"

RoadMaterial::RoadMaterial() {
    textureNames = {"Rock", "Road", "Cracks"};
    textureFiltering = {QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear};
}

void RoadMaterial::changeFilterType(const TextrureType texture, const FilteringType filt) {
    textureFiltering[texture] = possibleFilters[filt];
    if (textures[texture]) {
        textures[texture]->setMinMagFilters(possibleFilters[filt], possibleFilters[filt]);
        if (possibleFilters[filt] == QOpenGLTexture::LinearMipMapLinear) {
            textures[texture]->generateMipMaps();
        }
    }
}

void RoadMaterial::updateTextures() {
    for (size_t i = 0; i < textures.size(); ++i) {
        if (textures[i]) {
            glActiveTexture(GL_TEXTURE0 + i);
            textures[i]->bind();
            std::string uniformName = "tex" + textureNames[i];
            program->setUniformValue(uniformName.c_str(), static_cast<int>(i));
        }
    }
}

void RoadMaterial::initTextures() {
    QImage rockImage(":/road_2.jpg");
    textures.emplace_back(std::make_shared<QOpenGLTexture>(rockImage.mirrored()));
    textures[ROCK]->setWrapMode(QOpenGLTexture::Repeat);
    textures[ROCK]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[ROCK]->generateMipMaps();

    QImage roadImage(":/road_1.jpg");
    textures.emplace_back(std::make_shared<QOpenGLTexture>(roadImage.mirrored()));
    textures[ROAD]->setWrapMode(QOpenGLTexture::Repeat);
    textures[ROAD]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[ROAD]->generateMipMaps();

    QImage cracksImage(":/road_3.jpg");
    textures.emplace_back(std::make_shared<QOpenGLTexture>(cracksImage.mirrored()));
    textures[CRACKS]->setWrapMode(QOpenGLTexture::Repeat);
    textures[CRACKS]->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::LinearMipMapLinear);
    textures[CRACKS]->generateMipMaps();
}

void RoadMaterial::setTotalMatrix(const QMatrix4x4& newTotalMatrix) {
    totalMatrix = newTotalMatrix;
}

void RoadMaterial::initialize(QOpenGLShaderProgram* prog) {
    initializeOpenGLFunctions();
    program = prog;
    initUniforms();
    initTextures();
    update();
}

void RoadMaterial::render() {
    program->bind();
    setUniforms();
    updateTextures();
    glVertexAttribPointer(posAttr, 3, GL_FLOAT, GL_FALSE, 0, vertices.data());
    glEnableVertexAttribArray(posAttr);
    glDrawElements(GL_TRIANGLE_STRIP, indices.size(), GL_UNSIGNED_INT, indices.data());
    glDisableVertexAttribArray(posAttr);
    program->release();
}

void RoadMaterial::initUniforms() {
    posAttr = program->attributeLocation("posAttr");
    normalUniform = program->uniformLocation("normal");
    totalMatrixUniform = program->uniformLocation("totalMatrix");
    widhtUniform = program->uniformLocation("xSize");
    heightUniform = program->uniformLocation("zSize");
}

void RoadMaterial::setUniforms() const {
    program->setUniformValue(normalUniform, normal);
    program->setUniformValue(totalMatrixUniform, totalMatrix);
    program->setUniformValue(widhtUniform, widht);
    program->setUniformValue(heightUniform, height / 100);
}

void RoadMaterial::update() {
    const float halfWidth = widht / 2.0f;
    vertices[0] = QVector3D(-halfWidth, 0.0f, 0.0f);
    vertices[1] = QVector3D(halfWidth, 0.0f, 0.0f);
    vertices[2] = QVector3D(-halfWidth, 0.0f, height);
    vertices[3] = QVector3D(halfWidth, 0.0f, height);
}
