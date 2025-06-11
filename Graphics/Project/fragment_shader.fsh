#version 130

varying vec3 fragNormal;
varying vec3 fragPos;

uniform vec3 materialColor;
uniform vec3 lightDir;
uniform bool directedLightEnabled;
uniform vec3 cameraPos;
uniform vec3 lightPos;

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 lightDirNorm = normalize(-lightDir);
    float diff = directedLightEnabled ? max(dot(norm, lightDirNorm), 0.0) : 0.0;

    float ambientFactor = 0.3 + 0.2 * max(norm.y, 0.0);

    vec3 viewDir = normalize(cameraPos - fragPos);
    vec3 reflectDir = reflect(-lightDirNorm, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0) * 0.5;

    vec3 ambient = materialColor * ambientFactor;
    vec3 diffuse = materialColor * diff;
    vec3 specular = vec3(1.0) * spec;

    gl_FragColor = vec4(ambient + diffuse + specular, 1.0);
}
