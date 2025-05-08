#version 130

attribute highp vec4 posAttr;
attribute highp vec3 normAttr;

uniform highp mat4 projectionMatrix;
uniform highp mat3 normMatrix;
uniform highp mat4 model;
uniform highp mat4 viewMatrix;
uniform highp float morphing;
uniform highp vec3 lightPos;
uniform highp vec3 lightColor;
uniform highp vec3 viewPos;
uniform int lightType;
uniform float lightIntensity;
uniform float lightAttenuation;
uniform vec3 lightDirection;
uniform float lightCutoff;
uniform float outerCutoff;
uniform float ambientStrength;
uniform float diffuseStrength;
uniform float specularStrength;

varying highp vec3 vLightPos;
varying highp vec3 normVec;
varying highp vec3 fragPos;
varying highp vec3 vViewPos;
varying highp vec3 vLightColor;

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

    vec3 ambient = ambientStrength * lightColor;

    vec3 lightDir;
    float attenuation = 1.0;
    if (lightType == 0) {
        lightDir = normalize(lightDirection);
    }
    else {
        lightDir = normalize(lightPos - fragPos);
        float distance = length(lightPos - fragPos);
        attenuation = lightIntensity / (1.0 + lightAttenuation * distance * distance);
    }

    float spotlightEffect = 1.0;
    if (lightType == 2) {
        float theta = dot(lightDir, normalize(-lightDirection));
        float epsilon = lightCutoff - outerCutoff;
        spotlightEffect = clamp((theta - outerCutoff) / epsilon, 0.0, 1.0);
    }

    float diff = max(dot(normVec, lightDir), 0.0);
    vec3 diffuse = diffuseStrength * diff * lightColor;

    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, normVec);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * lightColor;

    vLightColor = ambient + (diffuse + specular) * attenuation * spotlightEffect;
}
