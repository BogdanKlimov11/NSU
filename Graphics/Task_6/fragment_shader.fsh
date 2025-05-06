#version 130

in vec3 fragPos;
in vec2 texCoord;
in vec3 normal;
in vec3 tangent;
in vec3 bitangent;

out vec4 fragColor;

uniform sampler2D dayTexture;
uniform sampler2D nightTexture;
uniform sampler2D normalMap;
uniform int useTexture;
uniform int useNormalMap;
uniform int useNightTexture;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform mat3 normMatrix;

void main() {
    vec3 norm = normalize(normal);
    if (useNormalMap == 1) {
        vec3 normalMapColor = texture2D(normalMap, texCoord).rgb;
        normalMapColor = normalMapColor * 2.0 - 1.0;
        mat3 TBN = mat3(normalize(tangent), normalize(bitangent), norm);
        norm = normalize(TBN * normalMapColor);
    }

    vec3 lightDir = normalize(lightPos - fragPos);
    float diff = max(dot(norm, lightDir), 0.0);

    vec3 diffuseColor = vec3(0.5, 0.5, 0.5);
    if (useTexture == 1) {
        if (useNightTexture == 1) {
            diffuseColor = texture2D(nightTexture, texCoord).rgb * 3.0;
        } else {
            diffuseColor = texture2D(dayTexture, texCoord).rgb;
        }
    }

    vec3 ambient = 0.1 * diffuseColor;
    vec3 diffuse = diff * diffuseColor;

    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = 0.2 * spec * vec3(1.0, 1.0, 1.0);

    vec3 result = ambient + diffuse + specular;
    fragColor = vec4(result, 1.0);
}
