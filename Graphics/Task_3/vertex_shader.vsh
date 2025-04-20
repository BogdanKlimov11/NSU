attribute highp vec4 posAttr;
attribute highp vec3 normAttr;

uniform highp mat4 projection_matrix;
uniform highp mat3 norm_matrix;
uniform highp mat4 model;
uniform highp mat4 view_matrix;
uniform highp float morph_param;
uniform highp vec3 lightPos;
uniform highp vec3 lightColor;
uniform highp vec3 viewPos;
uniform int lightType;
uniform float lightIntensity;
uniform float lightAttenuation;
uniform vec3 lightDirection;
uniform float lightCutoff;

uniform vec3 materialAmbient;
uniform vec3 materialDiffuse;
uniform vec3 materialSpecular;
uniform vec3 globalAmbient;

varying highp vec3 v_lightPos;
varying highp vec3 norm_vec;
varying highp vec3 frag_pos;
varying highp vec3 v_viewPos;
varying highp vec3 v_light_color;

void main() {
    float x = posAttr.x;
    float y = posAttr.y;
    float z = posAttr.z;
    float rad = sqrt(x*x + y*y + z*z);
    float phi = atan(y, x);
    float theta = acos(z / rad);
    float norm_rad = 1.0;
    float morph_rad = morph_param * rad + (1.0 - morph_param) * norm_rad;
    vec4 morph_posit = vec4(morph_rad * sin(theta) * cos(phi), morph_rad * sin(theta) * sin(phi), morph_rad * cos(theta), 1.0);
    gl_Position = projection_matrix * view_matrix * morph_posit;

    v_viewPos = viewPos;
    v_lightPos = lightPos;

    vec3 spherical_normal = normalize(vec3(morph_posit.x, morph_posit.y, morph_posit.z));
    norm_vec = normalize(norm_matrix * mix(normAttr, spherical_normal, 1.0 - morph_param));

    frag_pos = vec3(model * posAttr);

    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * globalAmbient * materialAmbient;

    vec3 lightDir;
    float attenuation = 1.0;

    if (lightType == 0) {
        lightDir = normalize(lightDirection);
    } else {
        lightDir = normalize(lightPos - frag_pos);
        float distance = length(lightPos - frag_pos);
        attenuation = lightIntensity / (1.0 + lightAttenuation * distance * distance);
    }

    float spotlightEffect = 1.0;
    if (lightType == 2) {
        float theta = dot(lightDir, normalize(-lightDirection));
        spotlightEffect = (theta > cos(lightCutoff * 3.14159 / 180.0)) ? 1.0 : 0.0;
    }

    float diff = max(dot(norm_vec, lightDir), 0.0);
    vec3 diffuse = diff * lightColor * materialDiffuse;

    vec3 viewDir = normalize(viewPos - frag_pos);
    vec3 reflectDir = reflect(-lightDir, norm_vec);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = 0.5 * spec * lightColor * materialSpecular;

    v_light_color = ambient + (diffuse + specular) * attenuation * spotlightEffect;

    v_lightPos = lightPos;
    v_viewPos = viewPos;
}
