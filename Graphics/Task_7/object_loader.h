#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QVector2D>
#include <QVector3D>
#include <QString>
#include <vector>

struct Material {
    QString name;
    QVector3D ambientColor;
    QVector3D diffuseColor;
    QVector3D specularColor;
    float shininess = 0.0f;
    float transparency = 1.0f;
    int illumModel = 0;
    QString diffuseTexture;
};

struct Vertex {
    QVector3D objectPosition;
    QVector3D objectNormal;
    QVector2D objectTexture;

    Vertex(const QVector3D& position, const QVector3D& normal, const QVector2D& texture)
        : objectPosition(position), objectNormal(normal), objectTexture(texture) {}
};

class Object : protected QOpenGLFunctions {
public:
    Object(const QString& name, const QString& filePath, float scale = 1.0f);
    ~Object();

    void initialize();
    void render(QOpenGLShaderProgram& program);
    bool isValid() const { return objectValid; }
    QVector3D getMaterialColor() const { return materials.empty() ? QVector3D(1.0f, 1.0f, 1.0f) : materials[0].diffuseColor; }
    void setMaterialColor(const QVector3D& color);

private:
    bool loadMaterial(const QString& materialPath);
    bool loadObject(const QString& objectPath, float scale);
    void loadTextures();

    struct MeshPart {
        std::vector<Vertex> vertices;
        std::vector<GLint> indices;
        size_t materialIndex;
        QOpenGLBuffer vbo;
        QOpenGLBuffer ibo;
    };

    QString objectName;
    std::vector<MeshPart> meshParts;
    std::vector<Material> materials;
    std::vector<QOpenGLTexture*> textures;
    bool objectValid = false;
};
