// Униформная переменная для цвета объекта (RGB)
uniform vec3 objectColor;
// Униформная переменная для цвета источника света (RGB)
uniform vec3 lightColor;

// Вариационные переменные для передачи данных из вершинного шейдера
varying vec3 v_lightPos; // Позиция источника света
varying highp vec3 frag_pos; // Позиция фрагмента в мировых координатах
varying highp vec3 norm_vec; // Нормаль фрагмента
varying highp vec3 v_viewPos; // Позиция камеры (точки обзора)
varying highp vec3 v_light_color; // Цвет света, вычисленный в вершинном шейдере

// Главная функция фрагментного шейдера
void main() {
    // Установка цвета фрагмента: произведение цвета объекта и цвета света (v_light_color),
    // с полной непрозрачностью (alpha = 1.0)
    gl_FragColor = vec4(objectColor * v_light_color, 1.0f);
}
