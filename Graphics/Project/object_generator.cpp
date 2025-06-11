#include <cmath>

#include "billiard_widget.h"

void BilliardWidget::generateSphere(Ball& ball, float radius, int slices, int stacks) {
    ball.vertices.clear();
    ball.normals.clear();
    ball.indices.clear();

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * i / stacks;
        float sinPhi = sin(phi);
        float cosPhi = cos(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * j / slices;
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);

            float x = radius * sinPhi * cosTheta;
            float y = radius * sinPhi * sinTheta;
            float z = radius * cosPhi;
            ball.vertices.push_back(QVector3D(x, y, z));
            ball.normals.push_back(QVector3D(x, y, z).normalized());
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int k1 = i * (slices + 1) + j;
            int k2 = k1 + slices + 1;

            ball.indices.push_back(k1);
            ball.indices.push_back(k2);
            ball.indices.push_back(k1 + 1);

            ball.indices.push_back(k2);
            ball.indices.push_back(k2 + 1);
            ball.indices.push_back(k1 + 1);
        }
    }
}

void BilliardWidget::generateCue(Cue& cue, float baseRadius, float topRadius, float height, int sides) {
    cue.vertices.clear();
    cue.normals.clear();
    cue.indices.clear();

    int baseIndex = cue.vertices.size();
    for (int i = 0; i <= sides; ++i) {
        float theta = 2.0f * M_PI * i / sides;
        float xBase = baseRadius * cos(theta);
        float zBase = baseRadius * sin(theta);
        cue.vertices.push_back(QVector3D(0.0f, xBase, zBase));
        cue.normals.push_back(QVector3D(0.0f, cos(theta), sin(theta)).normalized());
    }

    int topIndex = cue.vertices.size();
    for (int i = 0; i <= sides; ++i) {
        float theta = 2.0f * M_PI * i / sides;
        float xTop = topRadius * cos(theta);
        float zTop = topRadius * sin(theta);
        cue.vertices.push_back(QVector3D(height, xTop, zTop));
        cue.normals.push_back(QVector3D(0.0f, cos(theta), sin(theta)).normalized());
    }

    for (int i = 0; i < sides; ++i) {
        int next = (i + 1) % sides;
        cue.indices.push_back(baseIndex + i);
        cue.indices.push_back(baseIndex + next);
        cue.indices.push_back(topIndex + next);

        cue.indices.push_back(baseIndex + i);
        cue.indices.push_back(topIndex + next);
        cue.indices.push_back(topIndex + i);
    }

    cue.vbo.create();
    cue.vbo.bind();
    cue.vbo.allocate(cue.vertices.data(), cue.vertices.size() * sizeof(QVector3D));
    cue.vbo.release();

    cue.nbo.create();
    cue.nbo.bind();
    cue.nbo.allocate(cue.normals.data(), cue.normals.size() * sizeof(QVector3D));
    cue.nbo.release();

    cue.ibo.create();
    cue.ibo.bind();
    cue.ibo.allocate(cue.indices.data(), cue.indices.size() * sizeof(GLuint));
    cue.ibo.release();
}

void BilliardWidget::generateTableBorders() {
    float w = 8.91f, h = 4.46f;
    float borderHeight = 0.2f;
    float borderThickness = 0.2f;
    float pocketRadius = 0.15f;
    float sideCut = 0.4f;
    tableBorders.vertices.clear();
    tableBorders.normals.clear();
    tableBorders.indices.clear();

    auto addQuad = [&](QVector3D v0, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D normal) {
        int base = tableBorders.vertices.size();
        tableBorders.vertices.push_back(v0);
        tableBorders.vertices.push_back(v1);
        tableBorders.vertices.push_back(v2);
        tableBorders.vertices.push_back(v3);
        for (int i = 0; i < 4; ++i) tableBorders.normals.push_back(normal);
        tableBorders.indices.push_back(base);
        tableBorders.indices.push_back(base + 1);
        tableBorders.indices.push_back(base + 2);
        tableBorders.indices.push_back(base + 1);
        tableBorders.indices.push_back(base + 3);
        tableBorders.indices.push_back(base + 2);
    };

    addQuad(
        QVector3D(-w/2, 0.0f, -h/2),
        QVector3D(w/2, 0.0f, -h/2),
        QVector3D(-w/2, borderHeight, -h/2),
        QVector3D(w/2, borderHeight, -h/2),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f - pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(-w/2 + sideCut, 0.0f, -h/2 + borderThickness),
        QVector3D(0.0f - pocketRadius, 0.0f, -h/2 + borderThickness),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(0.0f + pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(w/2 - sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f + pocketRadius, 0.0f, -h/2 + borderThickness),
        QVector3D(w/2 - sideCut, 0.0f, -h/2 + borderThickness),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut/2, borderHeight, -h/2),
        QVector3D(0.0f - pocketRadius, borderHeight, -h/2),
        QVector3D(-w/2 + sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f - pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(0.0f + pocketRadius, borderHeight, -h/2),
        QVector3D(w/2 - sideCut/2, borderHeight, -h/2),
        QVector3D(0.0f + pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(w/2 - sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, h/2),
        QVector3D(w/2, 0.0f, h/2),
        QVector3D(-w/2, borderHeight, h/2),
        QVector3D(w/2, borderHeight, h/2),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(0.0f - pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(-w/2 + sideCut, 0.0f, h/2 - borderThickness),
        QVector3D(0.0f - pocketRadius, 0.0f, h/2 - borderThickness),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(0.0f + pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(w/2 - sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(0.0f + pocketRadius, 0.0f, h/2 - borderThickness),
        QVector3D(w/2 - sideCut, 0.0f, h/2 - borderThickness),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(0.0f - pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(-w/2 + sideCut/2, borderHeight, h/2),
        QVector3D(0.0f - pocketRadius, borderHeight, h/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(0.0f + pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(w/2 - sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(0.0f + pocketRadius, borderHeight, h/2),
        QVector3D(w/2 - sideCut/2, borderHeight, h/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, -h/2),
        QVector3D(-w/2, 0.0f, h/2),
        QVector3D(-w/2, borderHeight, -h/2),
        QVector3D(-w/2, borderHeight, h/2),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(-w/2 + borderThickness, 0.0f, -h/2 + sideCut),
        QVector3D(-w/2 + borderThickness, 0.0f, h/2 - sideCut),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(-w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, -h/2),
        QVector3D(w/2, 0.0f, h/2),
        QVector3D(w/2, borderHeight, -h/2),
        QVector3D(w/2, borderHeight, h/2),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(w/2 - borderThickness, 0.0f, -h/2 + sideCut),
        QVector3D(w/2 - borderThickness, 0.0f, h/2 - sideCut),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut, 0.0f, h/2 - borderThickness),
        QVector3D(-w/2 + sideCut/2, 0.0f, h/2),
        QVector3D(-w/2 + sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(-w/2 + sideCut/2, borderHeight, h/2),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut, 0.0f, h/2 - borderThickness),
        QVector3D(w/2 - sideCut/2, 0.0f, h/2),
        QVector3D(w/2 - sideCut, borderHeight, h/2 - borderThickness),
        QVector3D(w/2 - sideCut/2, borderHeight, h/2),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(pocketRadius, 0.0f, h/2 - borderThickness),
        QVector3D(pocketRadius, 0.0f, h/2),
        QVector3D(pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(pocketRadius, borderHeight, h/2),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-pocketRadius, 0.0f, h/2 - borderThickness),
        QVector3D(-pocketRadius, 0.0f, h/2),
        QVector3D(-pocketRadius, borderHeight, h/2 - borderThickness),
        QVector3D(-pocketRadius, borderHeight, h/2),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut, 0.0f, -h/2 + borderThickness),
        QVector3D(-w/2 + sideCut/2, 0.0f, -h/2),
        QVector3D(-w/2 + sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(-w/2 + sideCut/2, borderHeight, -h/2),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut, 0.0f, -h/2 + borderThickness),
        QVector3D(w/2 - sideCut/2, 0.0f, -h/2),
        QVector3D(w/2 - sideCut, borderHeight, -h/2 + borderThickness),
        QVector3D(w/2 - sideCut/2, borderHeight, -h/2),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(pocketRadius, 0.0f, -h/2 + borderThickness),
        QVector3D(pocketRadius, 0.0f, -h/2),
        QVector3D(pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(pocketRadius, borderHeight, -h/2),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-pocketRadius, 0.0f, -h/2 + borderThickness),
        QVector3D(-pocketRadius, 0.0f, -h/2),
        QVector3D(-pocketRadius, borderHeight, -h/2 + borderThickness),
        QVector3D(-pocketRadius, borderHeight, -h/2),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, h/2 - sideCut/2),
        QVector3D(w/2 - borderThickness, 0.0f, h/2 - sideCut),
        QVector3D(w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, -h/2 + sideCut/2),
        QVector3D(w/2 - borderThickness, 0.0f, -h/2 + sideCut),
        QVector3D(w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, h/2 - sideCut/2),
        QVector3D(-w/2 + borderThickness, 0.0f, h/2 - sideCut),
        QVector3D(-w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 - sideCut),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, -h/2 + sideCut/2),
        QVector3D(-w/2 + borderThickness, 0.0f, -h/2 + sideCut),
        QVector3D(-w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 + sideCut),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut/2, 0.0f, h/2),
        QVector3D(w/2, 0.0f, h/2 - sideCut/2),
        QVector3D(w/2 - sideCut/2, borderHeight, h/2),
        QVector3D(w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(-1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut/2, borderHeight, h/2),
        QVector3D(w/2, borderHeight, h/2),
        QVector3D(w/2 - sideCut/4, borderHeight, h/2 - sideCut/4),
        QVector3D(w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut/2, 0.0f, -h/2),
        QVector3D(w/2, 0.0f, -h/2 + sideCut/2),
        QVector3D(w/2 - sideCut/2, borderHeight, -h/2),
        QVector3D(w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(-1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2 - sideCut/2, borderHeight, -h/2),
        QVector3D(w/2, borderHeight, -h/2),
        QVector3D(w/2 - sideCut/4, borderHeight, -h/2 + sideCut/4),
        QVector3D(w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut/2, 0.0f, -h/2),
        QVector3D(-w/2, 0.0f, -h/2 + sideCut/2),
        QVector3D(-w/2 + sideCut/2, borderHeight, -h/2),
        QVector3D(-w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut/2, borderHeight, -h/2),
        QVector3D(-w/2, borderHeight, -h/2),
        QVector3D(-w/2 + sideCut/4, borderHeight, -h/2 + sideCut/4),
        QVector3D(-w/2, borderHeight, -h/2 + sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut/2, 0.0f, h/2),
        QVector3D(-w/2, 0.0f, h/2 - sideCut/2),
        QVector3D(-w/2 + sideCut/2, borderHeight, h/2),
        QVector3D(-w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 + sideCut/2, borderHeight, h/2),
        QVector3D(-w/2, borderHeight, h/2),
        QVector3D(-w/2 + sideCut/4, borderHeight, h/2 - sideCut/4),
        QVector3D(-w/2, borderHeight, h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-sideCut, 0.0f, h/2),
        QVector3D(-sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(-sideCut, borderHeight, h/2),
        QVector3D(-sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(-1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(-sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(sideCut, 0.0f, h/2),
        QVector3D(sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(sideCut, borderHeight, h/2),
        QVector3D(sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-sideCut, 0.0f, h/2),
        QVector3D(sideCut, 0.0f, h/2),
        QVector3D(-sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(sideCut/2, 0.0f, h/2 + sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-sideCut, borderHeight, h/2),
        QVector3D(sideCut, borderHeight, h/2),
        QVector3D(-sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(sideCut/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-sideCut, 0.0f, -h/2),
        QVector3D(-sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(-sideCut, borderHeight, -h/2),
        QVector3D(-sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(-1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(-sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(sideCut, 0.0f, -h/2),
        QVector3D(sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(sideCut, borderHeight, -h/2),
        QVector3D(sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-sideCut, 0.0f, -h/2),
        QVector3D(sideCut, 0.0f, -h/2),
        QVector3D(-sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(sideCut/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-sideCut, borderHeight, -h/2),
        QVector3D(sideCut, borderHeight, -h/2),
        QVector3D(-sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(sideCut/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, 0.0f, -h/2),
        QVector3D(w/2 - borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2 - 2*sideCut, borderHeight, -h/2),
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(-1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2 - borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2 + borderThickness, 0.0f, -h/2),
        QVector3D(w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2),
        QVector3D(1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2 + borderThickness, 0.0f, -h/2),
        QVector3D(w/2 + borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 + borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(w/2, 0.0f, -h/2 + 2*sideCut),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(w/2, borderHeight, -h/2 + 2*sideCut),
        QVector3D(1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, 0.0f, -h/2),
        QVector3D(w/2 - borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2, 0.0f, -h/2),
        QVector3D(w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(w/2, 0.0f, -h/2 + 2*sideCut),
        QVector3D(w/2 + borderThickness, 0.0f, -h/2),
        QVector3D(w/2 + borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, borderHeight, -h/2),
        QVector3D(w/2 - borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(w/2, borderHeight, -h/2),
        QVector3D(w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(w/2, borderHeight, -h/2 + 2*sideCut),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2),
        QVector3D(w/2 + borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, 0.0f, -h/2),
        QVector3D(-w/2 + borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2 + 2*sideCut, borderHeight, -h/2),
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 + borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(-w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2),
        QVector3D(-w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2),
        QVector3D(-1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2),
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(-w/2, 0.0f, -h/2 + 2*sideCut),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(-w/2, borderHeight, -h/2 + 2*sideCut),
        QVector3D(-1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, 0.0f, -h/2),
        QVector3D(-w/2 + borderThickness, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2, 0.0f, -h/2),
        QVector3D(-w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, -h/2 - sideCut/2),
        QVector3D(-w/2, 0.0f, -h/2 + 2*sideCut),
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2),
        QVector3D(-w/2 - borderThickness, 0.0f, -h/2 + borderThickness),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, borderHeight, -h/2),
        QVector3D(-w/2 + borderThickness, borderHeight, -h/2 - sideCut/2),
        QVector3D(-w/2, borderHeight, -h/2),
        QVector3D(-w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, borderHeight, -h/2 - sideCut/2),
        QVector3D(-w/2, borderHeight, -h/2 + 2*sideCut),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2),
        QVector3D(-w/2 - borderThickness, borderHeight, -h/2 + borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, 0.0f, h/2),
        QVector3D(w/2 - borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2 - 2*sideCut, borderHeight, h/2),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(-1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2 - borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2 + borderThickness, 0.0f, h/2),
        QVector3D(w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(w/2 + borderThickness, borderHeight, h/2),
        QVector3D(1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2 + borderThickness, 0.0f, h/2),
        QVector3D(w/2 + borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(w/2 + borderThickness, borderHeight, h/2),
        QVector3D(w/2 + borderThickness, borderHeight, h/2 - borderThickness),
        QVector3D(1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 + borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(w/2, 0.0f, h/2 - 2*sideCut),
        QVector3D(w/2 + borderThickness, borderHeight, h/2 - borderThickness),
        QVector3D(w/2, borderHeight, h/2 - 2*sideCut),
        QVector3D(1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, 0.0f, h/2),
        QVector3D(w/2 - borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2, 0.0f, h/2),
        QVector3D(w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(w/2, 0.0f, h/2 - 2*sideCut),
        QVector3D(w/2 + borderThickness, 0.0f, h/2),
        QVector3D(w/2 + borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - 2*sideCut, borderHeight, h/2),
        QVector3D(w/2 - borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(w/2, borderHeight, h/2),
        QVector3D(w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(w/2, borderHeight, h/2 - 2*sideCut),
        QVector3D(w/2 + borderThickness, borderHeight, h/2),
        QVector3D(w/2 + borderThickness, borderHeight, h/2 - borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, 0.0f, h/2),
        QVector3D(-w/2 + borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2 + 2*sideCut, borderHeight, h/2),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(1.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2 + borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(-w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 0.0f, 1.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2 - borderThickness, 0.0f, h/2),
        QVector3D(-w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(-w/2 - borderThickness, borderHeight, h/2),
        QVector3D(-1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 - borderThickness, 0.0f, h/2),
        QVector3D(-w/2 - borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(-w/2 - borderThickness, borderHeight, h/2),
        QVector3D(-w/2 - borderThickness, borderHeight, h/2 - borderThickness),
        QVector3D(-1.0f, 0.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 - borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(-w/2, 0.0f, h/2 - 2*sideCut),
        QVector3D(-w/2 - borderThickness, borderHeight, h/2 - borderThickness),
        QVector3D(-w/2, borderHeight, h/2 - 2*sideCut),
        QVector3D(-1.0f, 0.0f, -1.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, 0.0f, h/2),
        QVector3D(-w/2 + borderThickness, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2, 0.0f, h/2),
        QVector3D(-w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, 0.0f, h/2 + sideCut/2),
        QVector3D(-w/2, 0.0f, h/2 - 2*sideCut),
        QVector3D(-w/2 - borderThickness, 0.0f, h/2),
        QVector3D(-w/2 - borderThickness, 0.0f, h/2 - borderThickness),
        QVector3D(0.0f, -1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + 2*sideCut, borderHeight, h/2),
        QVector3D(-w/2 + borderThickness, borderHeight, h/2 + sideCut/2),
        QVector3D(-w/2, borderHeight, h/2),
        QVector3D(-w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2, borderHeight, h/2 + sideCut/2),
        QVector3D(-w/2, borderHeight, h/2 - 2*sideCut),
        QVector3D(-(w/2 + borderThickness), borderHeight, h/2),
        QVector3D(-(w/2 + borderThickness), borderHeight, h/2 - borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
}

void BilliardWidget::generatePocketSquares() {
    float w = 8.91f;
    float h = 4.46f;
    float borderThickness = 0.2f;
    float pocketRadius = 0.15f;

    pocketSquare.vertices.clear();
    pocketSquare.normals.clear();
    pocketSquare.indices.clear();

    auto addQuad = [&](QVector3D v0, QVector3D v1, QVector3D v2, QVector3D v3, QVector3D normal) {
        int base = pocketSquare.vertices.size();
        pocketSquare.vertices.push_back(v0);
        pocketSquare.vertices.push_back(v1);
        pocketSquare.vertices.push_back(v2);
        pocketSquare.vertices.push_back(v3);
        for (int i = 0; i < 4; ++i) pocketSquare.normals.push_back(normal);
        pocketSquare.indices.push_back(base);
        pocketSquare.indices.push_back(base + 1);
        pocketSquare.indices.push_back(base + 2);
        pocketSquare.indices.push_back(base + 1);
        pocketSquare.indices.push_back(base + 3);
        pocketSquare.indices.push_back(base + 2);
    };

    addQuad(
        QVector3D(w/2 - borderThickness, 0.001f, -h/2),
        QVector3D(w/2, 0.001f, -h/2 + borderThickness),
        QVector3D(w/2 - 2*borderThickness, 0.001f, -h/2 + borderThickness),
        QVector3D(w/2 - borderThickness, 0.001f, -h/2 + 2*borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + borderThickness, 0.001f, -h/2),
        QVector3D(-w/2, 0.001f, -h/2 + borderThickness),
        QVector3D(-w/2 + 2*borderThickness, 0.001f, -h/2 + borderThickness),
        QVector3D(-w/2 + borderThickness, 0.001f, -h/2 + 2*borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(w/2 - borderThickness, 0.001f, h/2),
        QVector3D(w/2, 0.001f, h/2 - borderThickness),
        QVector3D(w/2 - 2*borderThickness, 0.001f, h/2 - borderThickness),
        QVector3D(w/2 - borderThickness, 0.001f, h/2 - 2*borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(-w/2 + borderThickness, 0.001f, h/2),
        QVector3D(-w/2, 0.001f, h/2 - borderThickness),
        QVector3D(-w/2 + 2*borderThickness, 0.001f, h/2 - borderThickness),
        QVector3D(-w/2 + borderThickness, 0.001f, h/2 - 2*borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(pocketRadius, 0.001f, h/2),
        QVector3D(-pocketRadius, 0.001f, h/2),
        QVector3D(pocketRadius, 0.001f, h/2 - borderThickness),
        QVector3D(-pocketRadius, 0.001f, h/2 - borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    addQuad(
        QVector3D(pocketRadius, 0.001f, -h/2),
        QVector3D(-pocketRadius, 0.001f, -h/2),
        QVector3D(pocketRadius, 0.001f, -h/2 + borderThickness),
        QVector3D(-pocketRadius, 0.001f, -h/2 + borderThickness),
        QVector3D(0.0f, 1.0f, 0.0f)
        );
}

void BilliardWidget::generateTableLegs() {
    float w = 8.91f, h = 4.46f;
    float legHeight = 1.0f;
    float legRadius = 0.25f;
    int sides = 8;
    float yOffset = -0.01f;
    tableLegs.vertices.clear();
    tableLegs.normals.clear();
    tableLegs.indices.clear();

    QVector3D legCenters[] = {
        QVector3D(-w/2 + legRadius, -legHeight/2 + yOffset, -h/2 + legRadius),
        QVector3D(w/2 - legRadius, -legHeight/2 + yOffset, -h/2 + legRadius),
        QVector3D(-w/2 + legRadius, -legHeight/2 + yOffset, h/2 - legRadius),
        QVector3D(w/2 - legRadius, -legHeight/2 + yOffset, h/2 - legRadius)
    };

    for (const auto& center : legCenters) {
        int base = tableLegs.vertices.size();
        for (int i = 0; i < sides; ++i) {
            float theta = 2.0f * M_PI * i / sides;
            float x = legRadius * cos(theta);
            float z = legRadius * sin(theta);
            tableLegs.vertices.push_back(center + QVector3D(x, -legHeight/2, z));
            tableLegs.vertices.push_back(center + QVector3D(x, legHeight/2, z));
            tableLegs.normals.push_back(QVector3D(cos(theta), 0.0f, sin(theta)));
            tableLegs.normals.push_back(QVector3D(cos(theta), 0.0f, sin(theta)));
        }

        for (int i = 0; i < sides; ++i) {
            int next = (i + 1) % sides;
            tableLegs.indices.push_back(base + i * 2);
            tableLegs.indices.push_back(base + next * 2);
            tableLegs.indices.push_back(base + i * 2 + 1);
            tableLegs.indices.push_back(base + next * 2);
            tableLegs.indices.push_back(base + next * 2 + 1);
            tableLegs.indices.push_back(base + i * 2 + 1);
        }

        int topCenter = tableLegs.vertices.size();
        int bottomCenter = topCenter + 1;
        tableLegs.vertices.push_back(center + QVector3D(0, legHeight/2, 0));
        tableLegs.vertices.push_back(center + QVector3D(0, -legHeight/2, 0));
        tableLegs.normals.push_back(QVector3D(0, 1, 0));
        tableLegs.normals.push_back(QVector3D(0, -1, 0));

        for (int i = 0; i < sides; ++i) {
            int next = (i + 1) % sides;
            tableLegs.indices.push_back(topCenter);
            tableLegs.indices.push_back(base + next * 2 + 1);
            tableLegs.indices.push_back(base + i * 2 + 1);
            tableLegs.indices.push_back(bottomCenter);
            tableLegs.indices.push_back(base + i * 2);
            tableLegs.indices.push_back(base + next * 2);
        }
    }
}

void BilliardWidget::generateShadow(Shadow& shadow, float radius, int segments) {
    shadow.vertices.clear();
    shadow.normals.clear();
    shadow.indices.clear();

    shadow.vertices.push_back(QVector3D(0.0f, 0.001f, 0.0f));
    shadow.normals.push_back(QVector3D(0.0f, 1.0f, 0.0f));

    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        shadow.vertices.push_back(QVector3D(x, 0.001f, z));
        shadow.normals.push_back(QVector3D(0.0f, 1.0f, 0.0f));
    }

    for (int i = 0; i < segments; ++i) {
        shadow.indices.push_back(0);
        shadow.indices.push_back(i + 1);
        shadow.indices.push_back((i + 1) % segments + 1);
    }
}
