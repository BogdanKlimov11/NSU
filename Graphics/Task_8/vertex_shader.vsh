#version 130

attribute highp vec4 posAttr;

uniform highp vec4 normal;
uniform highp mat4 totalMatrix;
uniform float xSize;
uniform float zSize;
varying vec4 norm;
varying vec4 globalPos;
varying vec2 uvRock;
varying vec2 uvRoad;
varying vec2 uvCracks;

void main() {
    vec2 uvBase = vec2(posAttr.x / xSize + 0.5, posAttr.z / zSize);
    float rockScale = 5.0;
    float roadScaleX = 1.0;
    float roadScaleZ = 1.0;
    float cracksScale = 3.0;

    uvRock = uvBase * vec2(rockScale, rockScale);
    uvRoad = uvBase * vec2(roadScaleX, roadScaleZ);
    uvCracks = uvBase * vec2(cracksScale, cracksScale);

    norm = normal;
    globalPos = totalMatrix * posAttr;
    gl_Position = totalMatrix * posAttr;
}
