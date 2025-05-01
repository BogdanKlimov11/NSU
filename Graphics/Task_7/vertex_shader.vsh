#version 130

attribute vec3 position;
attribute vec3 normal;
attribute vec2 texCoord;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 lightDir;
uniform bool lightEnabled;
uniform float ka;
uniform float kd;
uniform float ks;
uniform vec3 cameraPos;
uniform vec3 materialColor;

varying vec3 vColor;
varying vec2 vTexCoord;

void main() {
    vec4 worldPos = model * vec4(position, 1.0);
    gl_Position = projection * view * worldPos;

    vec3 normalWorld = normalize(mat3(model) * normal);
    vec3 lightDirection = normalize(lightDir);
    vec3 viewDir = normalize(cameraPos - worldPos.xyz);

    vec3 color = materialColor * ka;
    if (lightEnabled) {
        float diff = max(dot(normalWorld, -lightDirection), 0.0);
        color += materialColor * kd * diff;

        vec3 reflectDir = reflect(lightDirection, normalWorld);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
        color += vec3(1.0, 1.0, 1.0) * ks * spec;
    }

    vColor = clamp(color, 0.0, 1.0);
    vTexCoord = texCoord;
}
