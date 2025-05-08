#version 130

uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform int lightType;
uniform float lightIntensity;
uniform float lightAttenuation;
uniform vec3 lightDirection;
uniform float lightCutoff;
uniform float outerCutoff;
uniform float ambientStrength;
uniform float diffuseStrength;
uniform float specularStrength;

varying highp vec3 fragPos;
varying highp vec3 normVec;
varying highp vec3 vViewPos;
varying highp vec3 vLightPos;

void main() {
    vec3 norm = normalize(normVec);

    vec3 ambient = ambientStrength * lightColor;

    vec3 lightDir;
    float attenuation = 1.0;
    if (lightType == 0) {
        lightDir = normalize(lightDirection);
    }
    else {
        lightDir = normalize(vLightPos - fragPos);
        float distance = length(vLightPos - fragPos);
        attenuation = lightIntensity / (1.0 + lightAttenuation * distance * distance);
    }

    float spotlightEffect = 1.0;
    if (lightType == 2) {
        float theta = dot(lightDir, normalize(-lightDirection));
        float epsilon = lightCutoff - outerCutoff;
        spotlightEffect = clamp((theta - outerCutoff) / epsilon, 0.0, 1.0);
    }

    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diffuseStrength * diff * lightColor;

    vec3 viewDir = normalize(vViewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * lightColor;

    vec3 result = (ambient + (diffuse + specular) * attenuation * spotlightEffect) * objectColor;
    gl_FragColor = vec4(result, 1.0);
}
