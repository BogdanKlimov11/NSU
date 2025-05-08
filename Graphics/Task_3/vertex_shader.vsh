#version 130

attribute highp vec4 posAttr;
attribute highp vec3 normAttr;

uniform highp mat4 projectionMatrix;
uniform highp mat3 normMatrix;
uniform highp mat4 model;
uniform highp mat4 viewMatrix;
uniform highp float morphing;
uniform highp vec3 lightPos;
uniform highp vec3 viewPos;

varying highp vec3 vLightPos;
varying highp vec3 normVec;
varying highp vec3 fragPos;
varying highp vec3 vViewPos;

void main() {
    float x = posAttr.x;
    float y = posAttr.y;
    float z = posAttr.z;
    float rad = sqrt(x*x + y*y + z*z);
    float phi = atan(y, x);
    float theta = acos(z / rad);
    float normRad = 1.0;
    float morphRad = morphing * rad + (1.0 - morphing) * normRad;
    vec4 morphPosit = vec4(morphRad * sin(theta) * cos(phi), morphRad * sin(theta) * sin(phi), morphRad * cos(theta), 1.0);

    vec4 worldPos = model * morphPosit;

    gl_Position = projectionMatrix * viewMatrix * worldPos;

    fragPos = vec3(worldPos);

    vec3 sphericalNormal = normalize(vec3(morphPosit.x, morphPosit.y, morphPosit.z));
    normVec = normalize(normMatrix * mix(normAttr, sphericalNormal, 1.0 - morphing));

    vViewPos = viewPos;
    vLightPos = lightPos;
}
