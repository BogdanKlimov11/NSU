#version 130

varying vec4 norm;
varying vec4 globalPos;
varying vec2 uvRock;
varying vec2 uvRoad;
varying vec2 uvCracks;

uniform float mixParameter;
uniform float ambientIntensity;
uniform float diffuseIntensity;
uniform float specularIntensity;
uniform vec3 lightColor;
uniform vec3 ambientColor;
uniform vec3 specularColor;
uniform vec3 viewPos;
uniform vec3 lightPos;
uniform sampler2D texRock;
uniform sampler2D texRoad;
uniform sampler2D texCracks;

void main() {
    vec3 rockColor = texture2D(texRock, uvRock).rgb;
    vec3 roadColor = texture2D(texRoad, uvRoad).rgb;
    vec3 cracksColor = texture2D(texCracks, uvCracks).rgb;

    vec3 baseColor = mix(rockColor, cracksColor, mixParameter);
    vec3 finalColor = baseColor * roadColor;

    vec3 ambient = ambientIntensity * ambientColor;
    vec3 norm = normalize(norm.xyz);
    vec3 lightDirection = normalize(lightPos - globalPos.xyz);
    float diffuseValue = diffuseIntensity * max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diffuseValue * lightColor;
    vec3 viewDirection = normalize(viewPos - globalPos.xyz);
    vec3 reflectionDirection = reflect(-lightDirection, norm);
    float specularValue = pow(max(dot(viewDirection, reflectionDirection), 0.0), 32.0);
    vec3 specular = specularIntensity * specularValue * specularColor;

    finalColor = (ambient + diffuse + specular) * finalColor;

    gl_FragColor = vec4(finalColor, 1.0);
}
