// Атрибут позиции вершины (4 компоненты: x, y, z, w) с высокой точностью
attribute highp vec4 posAttr;
// Атрибут нормали вершины (3 компоненты: x, y, z) с высокой точностью
attribute highp vec3 normAttr;

// Униформные переменные для матриц трансформации
uniform highp mat4 projection_matrix; // Матрица проекции
uniform highp mat3 norm_matrix; // Матрица нормалей (для преобразования нормалей)
uniform highp mat4 model; // Модельная матрица
uniform highp mat4 view_matrix; // Матрица вида

// Униформные переменные для параметров морфинга и освещения
uniform highp float morph_param; // Параметр морфинга (0..1)
uniform highp vec3 lightPos; // Позиция источника света
uniform highp vec3 lightColor; // Цвет источника света (RGB)
uniform highp vec3 viewPos; // Позиция камеры
uniform int lightType; // Тип источника света (0 - направленный, 1 - точечный, 2 - прожектор)
uniform float lightIntensity; // Интенсивность света
uniform float lightAttenuation; // Коэффициент затухания света
uniform vec3 lightDirection; // Направление света
uniform float lightCutoff; // Угол обрезки для прожектора (в радианах)

// Вариационные переменные для передачи данных во фрагментный шейдер
varying highp vec3 v_lightPos; // Позиция источника света
varying highp vec3 norm_vec; // Нормаль фрагмента
varying highp vec3 frag_pos; // Позиция фрагмента в мировых координатах
varying highp vec3 v_viewPos; // Позиция камеры
varying highp vec3 v_light_color; // Цвет света с учетом освещения

// Главная функция вершинного шейдера
void main() {
    // Извлечение координат вершины из атрибута
    float x = posAttr.x;
    float y = posAttr.y;
    float z = posAttr.z;
    // Вычисление радиуса вершины относительно начала координат
    float rad = sqrt(x*x + y*y + z*z);
    // Вычисление угла φ (азимут) в сферических координатах
    float phi = atan(y, x);
    // Вычисление угла θ (полярный угол) в сферических координатах
    float theta = acos(z / rad);
    // Задание нормализованного радиуса (для сферической формы)
    float norm_rad = 1.0;
    // Линейная интерполяция радиуса между исходным (rad) и нормализованным (norm_rad)
    float morph_rad = morph_param * rad + (1.0 - morph_param) * norm_rad;
    // Вычисление новой позиции вершины в сферических координатах с учетом морфинга
    vec4 morph_posit = vec4(
        morph_rad * sin(theta) * cos(phi), // x-координата
        morph_rad * sin(theta) * sin(phi), // y-координата
        morph_rad * cos(theta), // z-координата
        1.0 // w-координата (гомогенная)
    );

    // Преобразование морфированной позиции в мировые координаты с помощью модельной матрицы
    vec4 world_pos = model * morph_posit;

    // Вычисление окончательной позиции вершины в пространстве отсечения
    gl_Position = projection_matrix * view_matrix * world_pos;

    // Передача позиции фрагмента в мировых координатах
    frag_pos = vec3(world_pos);

    // Вычисление нормали для сферической формы (нормализация морфированной позиции)
    vec3 spherical_normal = normalize(vec3(morph_posit.x, morph_posit.y, morph_posit.z));
    // Интерполяция между исходной нормалью (normAttr) и сферической нормалью на основе morph_param
    norm_vec = normalize(norm_matrix * mix(normAttr, spherical_normal, 1.0 - morph_param));

    // Передача позиции камеры и источника света во фрагментный шейдер
    v_viewPos = viewPos;
    v_lightPos = lightPos;

    // Задание силы фонового освещения
    float ambientStrength = 0.3;
    // Вычисление фоновой компоненты освещения
    vec3 ambient = ambientStrength * lightColor;

    // Объявление переменных для направления света и затухания
    vec3 lightDir;
    float attenuation = 1.0;

    // Вычисление направления света в зависимости от типа источника
    if (lightType == 0) {
        // Для направленного света используется нормализованное направление
        lightDir = normalize(lightDirection);
    } else {
        // Для точечного или прожекторного света направление вычисляется как вектор от фрагмента к источнику
        lightDir = normalize(lightPos - frag_pos);
        // Вычисление расстояния до источника света
        float distance = length(lightPos - frag_pos);
        // Вычисление затухания света (формула: интенсивность / (1 + затухание * расстояние^2))
        attenuation = lightIntensity / (1.0 + lightAttenuation * distance * distance);
    }

    // Задание эффекта прожектора (по умолчанию включен)
    float spotlightEffect = 1.0;
    if (lightType == 2) {
        // Вычисление угла между направлением света и направлением прожектора
        float theta = dot(lightDir, normalize(lightDirection));
        // Проверка, находится ли фрагмент внутри конуса прожектора
        spotlightEffect = (theta > lightCutoff) ? 1.0 : 0.0;
    }

    // Вычисление диффузной компоненты освещения
    float diff = max(dot(norm_vec, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Вычисление направления взгляда (от фрагмента к камере)
    vec3 viewDir = normalize(viewPos - frag_pos);
    // Вычисление направления отражения света
    vec3 reflectDir = reflect(-lightDir, norm_vec);
    // Вычисление зеркальной компоненты (степень 32 для резкости блика)
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    // Вычисление зеркальной компоненты с коэффициентом 0.5
    vec3 specular = 0.5 * spec * lightColor;

    // Итоговый цвет света: сумма фонового, диффузного и зеркального освещения с учетом затухания и эффекта прожектора
    v_light_color = ambient + (diffuse + specular) * attenuation * spotlightEffect;

    // Повторная передача позиции источника света и камеры (для совместимости с фрагментным шейдером)
    v_lightPos = lightPos;
    v_viewPos = viewPos;
}
