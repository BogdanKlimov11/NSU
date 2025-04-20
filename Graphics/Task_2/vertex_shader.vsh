// Атрибут позиции вершины (4 компоненты: x, y, z, w) с высокой точностью
attribute highp vec4 pos_attr;
// Атрибут цвета вершины (4 компоненты: r, g, b, a) с низкой точностью
attribute lowp vec4 col_attr;
// Вариационная переменная для передачи цвета во фрагментный шейдер (низкая точность)
varying lowp vec4 color;
// Униформная переменная для матрицы модель-вид-проекция (MVP) с высокой точностью
uniform highp mat4 mvp_matrix;
// Униформная переменная для параметра морфинга (диапазон 0..1) с низкой точностью
uniform lowp float morphing_param;

// Главная функция вершинного шейдера
void main() {
    // Нормализация позиции вершины (xyz) и масштабирование до 0.5 для создания сферической формы
    vec3 normalized_position = 0.5 * normalize(pos_attr.xyz);
    // Линейная интерполяция между исходной позицией и нормализованной на основе morphing_param
    vec3 morphed_position = mix(pos_attr.xyz, normalized_position, morphing_param);
    // Передача цвета вершины во фрагментный шейдер
    color = col_attr;
    // Вычисление окончательной позиции вершины в пространстве отсечения с применением матрицы MVP
    gl_Position = mvp_matrix * vec4(morphed_position, 1.0);
}
