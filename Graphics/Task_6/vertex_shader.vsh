// Атрибуты, передаваемые из программы (из VBO):
attribute vec3 posAttr; // Позиция вершины в локальных координатах
attribute vec3 normAttr; // Нормаль вершины в локальных координатах
attribute vec2 texAttr; // Текстурные координаты вершины
attribute vec3 tangentAttr; // Тангенс (касательная) к поверхности в точке вершины
attribute vec3 bitangentAttr; // Битангенс (биконсистентная перпендикулярная к тангенсу и нормали)

// Матрицы преобразований, передаваемые из программы:
uniform mat4 projection_matrix; // Матрица проекции (например, перспективной)
uniform mat4 model; // Модельная матрица — преобразует из локальных в мировые координаты
uniform mat4 view_matrix; // Видовая матрица — преобразует из мировых в координаты камеры
uniform mat3 norm_matrix; // Матрица для преобразования нормалей (обычно инверсно транспонированная от model)

// Переменные, которые будут переданы во фрагментный шейдер:
varying vec3 v_norm; // Преобразованная нормаль
varying vec2 v_text_coord; // Текстурные координаты
varying vec3 v_tangent; // Преобразованный тангенс
varying vec3 v_bitangent; // Преобразованный битангенс

void main() {
    // Преобразуем позицию вершины в координаты экрана (clip space)
    vec4 pos = vec4(posAttr, 1.0); // Расширяем до vec4 для матричных операций
    gl_Position = projection_matrix * view_matrix * model * pos;

    // Преобразуем нормаль, тангенс и битангенс в мировые/камерные координаты
    v_norm = norm_matrix * normAttr;
    v_tangent = norm_matrix * tangentAttr;
    v_bitangent = norm_matrix * bitangentAttr;

    // Передаём текстурные координаты напрямую — они не нуждаются в преобразовании
    v_text_coord = texAttr;
}
