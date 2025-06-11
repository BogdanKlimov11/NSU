#version 130

attribute vec3 position;
attribute vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 lightPos;

varying vec3 fragNormal;
varying vec3 fragPos;

void main() {
    fragPos = vec3(model * vec4(position, 1.0));
    fragNormal = normal;
    gl_Position = projection * view * model * vec4(position, 1.0);
}
