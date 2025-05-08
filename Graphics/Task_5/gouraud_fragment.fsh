#version 130

uniform vec3 objectColor;
uniform vec3 lightColor;

varying highp vec3 vLightPos;
varying highp vec3 fragPos;
varying highp vec3 normVec;
varying highp vec3 vViewPos;
varying highp vec3 vLightColor;

void main() {
    gl_FragColor = vec4(objectColor * vLightColor, 1.0);
}
