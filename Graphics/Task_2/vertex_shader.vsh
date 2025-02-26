attribute highp vec4 pos_attr;
attribute lowp vec4 col_attr;
varying lowp vec4 color;
uniform highp mat4 mvp_matrix;
uniform lowp float morphing_param;

void main() {
   vec3 normalized_position = 0.5 * normalize(pos_attr.xyz);
   vec3 morphed_position = mix(pos_attr, normalized_position, morphing_param);
   color = col_attr;
   gl_Position = mvp_matrix * vec4(morphed_position, 1.0);
}
