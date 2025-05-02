attribute highp vec4 posAttr;
attribute lowp vec4 colAttr;
varying lowp vec4 color;
uniform highp mat4 mvpMatrix;
uniform lowp float morphingParam;

void main() {
    vec3 normalizedPosition = 0.5 * normalize(posAttr.xyz);
    vec3 morphedPosition = mix(posAttr.xyz, normalizedPosition, morphingParam);
    color = colAttr;
    gl_Position = mvpMatrix * vec4(morphedPosition, 1.0);
}
