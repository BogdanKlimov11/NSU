#version 130

attribute highp vec4 posAttr;

uniform highp vec4 normal;
uniform highp mat4 totalMatrix;
uniform float xSize;
uniform float zSize;
varying vec4 norm;
varying vec4 globalPos;
varying vec2 uv;

void main() {
    uv = vec2(posAttr.x / xSize + 0.5, posAttr.z / zSize); // Упрощаем UV-координаты
    norm = normal;
    globalPos = totalMatrix * posAttr;
    gl_Position = totalMatrix * posAttr;
}
