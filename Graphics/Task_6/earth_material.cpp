#include <QImage>

#include "earth_material.h"

EarthMaterial::EarthMaterial() {
    QImage dayImage(":/day_map.png");
    if (!dayImage.isNull()) {
        dayTexture = std::make_shared<QOpenGLTexture>(dayImage);
        dayTexture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        dayTexture->setMagnificationFilter(QOpenGLTexture::Linear);
        dayTexture->setWrapMode(QOpenGLTexture::Repeat);
        dayTexture->generateMipMaps();
    }

    QImage nightImage(":/night_map.png");
    if (!nightImage.isNull()) {
        nightTexture = std::make_shared<QOpenGLTexture>(nightImage);
        nightTexture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        nightTexture->setMagnificationFilter(QOpenGLTexture::Linear);
        nightTexture->setWrapMode(QOpenGLTexture::Repeat);
        nightTexture->generateMipMaps();
    }

    QImage normalMapImage(":/normal_map.png");
    if (!normalMapImage.isNull()) {
        normalMap = std::make_shared<QOpenGLTexture>(normalMapImage);
        normalMap->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        normalMap->setMagnificationFilter(QOpenGLTexture::Linear);
        normalMap->setWrapMode(QOpenGLTexture::Repeat);
        normalMap->generateMipMaps();
    }
}

void EarthMaterial::bind(QOpenGLShaderProgram* program, bool useTexture, bool useNormalMap, bool useNightTexture) {
    if (useTexture && dayTexture && dayTexture->isCreated()) {
        dayTexture->bind(0);
        program->setUniformValue("dayTexture", 0);
    }
    if (useTexture && useNightTexture && nightTexture && nightTexture->isCreated()) {
        nightTexture->bind(1);
        program->setUniformValue("nightTexture", 1);
    }
    if (useNormalMap && normalMap && normalMap->isCreated()) {
        normalMap->bind(2);
        program->setUniformValue("normalMap", 2);
    }
}
