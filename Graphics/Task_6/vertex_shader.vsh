#version 130

in vec3 posAttr;
in vec3 normAttr;
in vec2 texAttr;
in vec3 tangentAttr;
in vec3 bitangentAttr;

out vec3 fragPos;
out vec2 texCoord;
out vec3 normal;
out vec3 tangent;
out vec3 bitangent;

uniform mat4 projectionMatrix;
uniform mat4 model;
uniform mat4 viewMatrix;

void main() {
    vec4 worldPos = model * vec4(posAttr, 1.0);
    fragPos = worldPos.xyz;
    texCoord = texAttr;
    normal = normalize(mat3(model) * normAttr);
    tangent = normalize(mat3(model) * tangentAttr);
    bitangent = normalize(mat3(model) * bitangentAttr);
    gl_Position = projectionMatrix * viewMatrix * worldPos;
}
