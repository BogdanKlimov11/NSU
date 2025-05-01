#include <QFile>
#include <QTextStream>
#include <QImage>

#include "object_loader.h"

Object::Object(const QString& name, const QString& filePath, float scale) : objectName(name) {
    loadObject(filePath, scale);
}

Object::~Object() {
    for (auto texture : textures) {
        delete texture;
    }
    for (auto& part : meshParts) {
        part.vbo.destroy();
        part.ibo.destroy();
    }
}

void Object::initialize() {
    initializeOpenGLFunctions();
    loadTextures();

    for (auto& part : meshParts) {
        part.vbo.create();
        part.vbo.bind();
        part.vbo.allocate(part.vertices.data(), part.vertices.size() * sizeof(Vertex));
        part.vbo.release();

        part.ibo.create();
        part.ibo.bind();
        part.ibo.allocate(part.indices.data(), part.indices.size() * sizeof(GLint));
        part.ibo.release();
    }

    objectValid = !meshParts.empty();
}

void Object::render(QOpenGLShaderProgram& program) {
    if (!objectValid) {
        return;
    }

    for (auto& part : meshParts) {
        if (part.vertices.empty()) {
            continue;
        }

        if (part.materialIndex < materials.size()) {
            program.setUniformValue("materialColor", materials[part.materialIndex].diffuseColor);
            program.setUniformValue("useTexture", part.materialIndex < textures.size() && textures[part.materialIndex]);
            if (part.materialIndex < textures.size() && textures[part.materialIndex]) {
                textures[part.materialIndex]->bind();
            }
        }

        part.vbo.bind();

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, objectPosition));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, objectNormal));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, objectTexture));

        glDrawArrays(GL_TRIANGLES, 0, part.vertices.size());

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);

        part.vbo.release();
    }
}

void Object::setMaterialColor(const QVector3D& color) {
    for (auto& material : materials) {
        material.diffuseColor = color;
    }
}

bool Object::loadMaterial(const QString& materialPath) {
    QFile file(materialPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    QTextStream in(&file);
    Material currentMaterial;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList tokens = line.split(" ", Qt::SkipEmptyParts);
        if (tokens.isEmpty()) continue;

        if (tokens[0] == "newmtl" && tokens.size() >= 2) {
            if (!currentMaterial.name.isEmpty()) {
                materials.push_back(currentMaterial);
            }
            currentMaterial = Material();
            currentMaterial.name = tokens[1];
        }
        else if (tokens[0] == "Ka" && tokens.size() >= 4) {
            float r = tokens[1].toFloat();
            float g = tokens[2].toFloat();
            float b = tokens[3].toFloat();
            currentMaterial.ambientColor = QVector3D(r, g, b);
        }
        else if (tokens[0] == "Kd" && tokens.size() >= 4) {
            float r = tokens[1].toFloat();
            float g = tokens[2].toFloat();
            float b = tokens[3].toFloat();
            currentMaterial.diffuseColor = QVector3D(r, g, b);
        }
        else if (tokens[0] == "Ks" && tokens.size() >= 4) {
            float r = tokens[1].toFloat();
            float g = tokens[2].toFloat();
            float b = tokens[3].toFloat();
            currentMaterial.specularColor = QVector3D(r, g, b);
        }
        else if (tokens[0] == "Ns" && tokens.size() >= 2) {
            currentMaterial.shininess = tokens[1].toFloat();
        }
        else if ((tokens[0] == "d" || tokens[0] == "Tr") && tokens.size() >= 2) {
            currentMaterial.transparency = tokens[1].toFloat();
        }
        else if (tokens[0] == "illum" && tokens.size() >= 2) {
            currentMaterial.illumModel = tokens[1].toInt();
        }
        else if (tokens[0] == "map_Kd" && tokens.size() >= 2) {
            currentMaterial.diffuseTexture = tokens[1];
        }
    }

    if (!currentMaterial.name.isEmpty()) {
        materials.push_back(currentMaterial);
    }

    file.close();
    return !materials.empty();
}

bool Object::loadObject(const QString& filePath, float scale) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    std::vector<QVector3D> tempPositions;
    std::vector<QVector3D> tempNormals;
    std::vector<QVector2D> tempTextures;
    QTextStream in(&file);
    size_t currentMaterialIndex = 0;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList tokens = line.split(" ", Qt::SkipEmptyParts);
        if (tokens.isEmpty()) continue;

        if (tokens[0] == "mtllib" && tokens.size() >= 2) {
            QString materialPath = ":/" + tokens[1];
            loadMaterial(materialPath);
        }
        else if (tokens[0] == "usemtl" && tokens.size() >= 2) {
            QString materialName = tokens[1];
            auto it = std::find_if(materials.begin(), materials.end(),
                                   [&materialName](const Material& mat) { return mat.name == materialName; });
            if (it != materials.end()) {
                currentMaterialIndex = std::distance(materials.begin(), it);
            }
        }
        else if (tokens[0] == "v" && tokens.size() >= 4) {
            float x = tokens[1].toFloat();
            float y = tokens[2].toFloat();
            float z = tokens[3].toFloat();
            tempPositions.push_back(QVector3D(x * scale, y * scale, z * scale));
        }
        else if (tokens[0] == "vn" && tokens.size() >= 4) {
            float x = tokens[1].toFloat();
            float y = tokens[2].toFloat();
            float z = tokens[3].toFloat();
            tempNormals.push_back(QVector3D(x, y, z));
        }
        else if (tokens[0] == "vt" && tokens.size() >= 3) {
            float u = tokens[1].toFloat();
            float v = tokens[2].toFloat();
            tempTextures.push_back(QVector2D(u, v));
        }
        else if (tokens[0] == "f" && tokens.size() >= 4) {
            MeshPart& currentPart = meshParts.empty() || meshParts.back().materialIndex != currentMaterialIndex
                                        ? meshParts.emplace_back(MeshPart{{}, {}, currentMaterialIndex})
                                        : meshParts.back();

            for (int i = 1; i <= 3; ++i) {
                QStringList indices = tokens[i].split("/");
                int vertexIndex = -1;
                int textureIndex = -1;
                int normalIndex = -1;

                if (indices.size() >= 1)
                    vertexIndex = indices[0].toInt() - 1;
                if (indices.size() >= 2 && !indices[1].isEmpty())
                    textureIndex = indices[1].toInt() - 1;
                if (indices.size() >= 3)
                    normalIndex = indices[2].toInt() - 1;

                if (vertexIndex < 0 || vertexIndex >= (int)tempPositions.size()) {
                    continue;
                }
                QVector3D position = tempPositions[vertexIndex];
                QVector3D normal = (normalIndex >= 0 && normalIndex < (int)tempNormals.size()) ? tempNormals[normalIndex] : QVector3D(0, 0, 1);
                QVector2D texture = (textureIndex >= 0 && textureIndex < (int)tempTextures.size()) ? tempTextures[textureIndex] : QVector2D(0, 0);

                currentPart.vertices.push_back(Vertex(position, normal, texture));
                currentPart.indices.push_back(currentPart.vertices.size() - 1);
            }
        }
    }

    file.close();
    return !meshParts.empty();
}

void Object::loadTextures() {
    initializeOpenGLFunctions();
    for (const auto& material : materials) {
        if (!material.diffuseTexture.isEmpty()) {
            QString texturePath = ":/" + material.diffuseTexture;
            QImage image(texturePath);
            if (image.isNull()) {
                textures.push_back(nullptr);
            } else {
                QOpenGLTexture* texture = new QOpenGLTexture(image);
                texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
                texture->setMagnificationFilter(QOpenGLTexture::Linear);
                texture->setWrapMode(QOpenGLTexture::ClampToEdge);
                textures.push_back(texture);
            }
        } else {
            textures.push_back(nullptr);
        }
    }
}
