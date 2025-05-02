#version 130

varying vec4 norm;
varying vec4 globalPos;
varying vec2 uv;

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
    // Комбинируем текстуры
    vec3 col = mix(texture2D(texRock, uv).rgb, texture2D(texRoad, uv).rgb, mixParameter);
    col = mix(col, texture2D(texCracks, uv).rgb, 0.5); // Уменьшаем влияние трещин

    // Освещение
    vec3 ambient = ambientIntensity * ambientColor;
    vec3 norm = normalize(norm.xyz);
    vec3 lightDirection = normalize(lightPos - globalPos.xyz);
    float diffuseValue = diffuseIntensity * max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diffuseValue * lightColor;
    vec3 viewDirection = normalize(viewPos - globalPos.xyz);
    vec3 reflectionDirection = reflect(-lightDirection, norm);
    float specularValue = pow(max(dot(viewDirection, reflectionDirection), 0.0), 32.0);
    vec3 specular = specularIntensity * specularValue * specularColor;

    // Итоговый цвет с освещением
    vec3 finalColor = (ambient + diffuse + specular) * col;

    // Для отладки: если освещение мешает, можно временно отключить его
    // gl_FragColor = vec4(col, 1.0); // Только текстуры, без освещения
    gl_FragColor = vec4(finalColor, 1.0);
}
