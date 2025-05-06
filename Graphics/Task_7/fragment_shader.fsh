#version 130

varying vec3 vColor;
varying vec2 vTexCoord;
uniform sampler2D diffuseTexture;
uniform bool useTexture;

void main() {
    vec4 color = vec4(vColor, 1.0);
    if (useTexture) {
        color *= texture2D(diffuseTexture, vTexCoord);
    }
    gl_FragColor = color;
}
