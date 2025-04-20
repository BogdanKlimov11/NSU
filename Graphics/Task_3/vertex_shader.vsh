// Атрибут позиции вершины (4 компоненты: x, y, z, w)
attribute highp vec4 posAttr;
// Атрибут нормали вершины (3 компоненты: x, y, z)
attribute highp vec3 normAttr;

// Униформная переменная для матрицы проекции
uniform highp mat4 projection_matrix;
// Униформная переменная для матрицы нормалей (для преобразования нормалей)
uniform highp mat3 norm_matrix;
// Униформная переменная для модельной матрицы
uniform highp mat4 model;
// Униформная переменная для матрицы вида
uniform highp mat4 view_matrix;
// Униформная переменная для параметра морфинга (диапазон 0..1)
uniform highp float morph_param;
// Униформная переменная для позиции источника света
uniform highp vec3 lightPos;
// Униформная переменная для цвета источника света (RGB)
uniform highp vec3 lightColor;
// Униформная переменная для позиции камеры
uniform highp vec3 viewPos;
// Униформная переменная для типа источника света (0 - направленный, 1 - точечный, 2 - прожектор)
uniform int lightType;
// Униформная переменная для интенсивности света
uniform float lightIntensity;
// Униформная переменная для затухания света
uniform float lightAttenuation;
// Униформная переменная для направления света
uniform vec3 lightDirection;
// Униформная переменная для угла обрезки света (в градусах, для прожектора)
uniform float lightCutoff;

// Униформные переменные для свойств материала
uniform vec3 materialAmbient; // Фоновое освещение материала
uniform vec3 materialDiffuse; // Диффузное освещение материала
uniform vec3 materialSpecular; // Зеркальное освещение материала
uniform vec3 globalAmbient; // Глобальное фоновое освещение сцены

// Вариационные переменные для передачи данных в фрагментный шейдер
varying highp vec3 v_lightPos; // Позиция источника света
varying highp vec3 norm_vec; // Нормаль фрагмента
varying highp vec3 frag_pos; // Позиция фрагмента
varying highp vec3 v_viewPos; // Позиция камеры
varying highp vec3 v_light_color; // Цвет света с учетом освещения

// Главная функция вершинного шейдера
void main() {
    // Извлечение координат вершины
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
    // Линейная интерполяция радиуса между исходным (rad) и нормализованным (norm_rad) на основе morph_param
    float morph_rad = morph_param * rad + (1.0 - morph_param) * norm_rad;
    // Вычисление новой позиции вершины в сферических координатах с учетом морфинга
    vec4 morph_posit = vec4(
        morph_rad * sin(theta) * cos(phi), // x-координата
        morph_rad * sin(theta) * sin(phi), // y-координата
        morph_rad * cos(theta), // z-координата
        1.0 // w-координата (гомогенная)
    );
    // Вычисление окончательной позиции вершины в пространстве отсечения
    gl_Position = projection_matrix * view_matrix * morph_posit;

    // Передача позиции камеры во фрагментный шейдер
    v_viewPos = viewPos;
    // Передача позиции источника света во фрагментный шейдер
    v_lightPos = lightPos;

    // Вычисление нормали для сферической формы (нормализация позиции морфированной вершины)
    vec3 spherical_normal = normalize(vec3(morph_posit.x, morph_posit.y, morph_posit.z));
    // Интерполяция между исходной нормалью (normAttr) и сферической нормалью на основе morph_param
    norm_vec = normalize(norm_matrix * mix(normAttr, spherical_normal, 1.0 - morph_param));

    // Вычисление позиции фрагмента в мировых координатах
    frag_pos = vec3(model * posAttr);

    // Задание силы фонового освещения
    float ambientStrength = 0.3;
    // Вычисление фонового освещения (глобальное фоновое * материал * сила)
    vec3 ambient = ambientStrength * globalAmbient * materialAmbient;

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
        // Вычисление расстояния от фрагмента до источника света
        float distance = length(lightPos - frag_pos);
        // Вычисление затухания света (формула: интенсивность / (1 + затухание * расстояние^2))
        attenuation = lightIntensity / (1.0 + lightAttenuation * distance * distance);
    }

    // Задание эффекта прожектора (по умолчанию включен)
    float spotlightEffect = 1.0;
    if (lightType == 2) {
        // Для прожекторного света вычисляется угол между направлением света и направлением прожектора
        float theta = dot(lightDir, normalize(-lightDirection));
        // Проверка, находится ли фрагмент внутри конуса прожектора (перевод угла обрезки в радианы)
        spotlightEffect = (theta > cos(lightCutoff * 3.14159 / 180.0)) ? 1.0 : 0.0;
    }

    // Вычисление диффузного освещения (максимум между скалярным произведением нормали и направления света и 0)
    float diff = max(dot(norm_vec, lightDir), 0.0);
    // Вычисление диффузной компоненты (диффузный коэффициент * цвет света * материал)
    vec3 diffuse = diff * lightColor * materialDiffuse;

    // Вычисление направления взгляда (от фрагмента к камере)
    vec3 viewDir = normalize(viewPos - frag_pos);
    // Вычисление направления отражения света
    vec3 reflectDir = reflect(-lightDir, norm_vec);
    // Вычисление зеркальной компоненты (степень 32 для резкости блика)
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    // Вычисление зеркальной компоненты (коэффициент 0.5 * зеркальный коэффициент * цвет света * материал)
    vec3 specular = 0.5 * spec * lightColor * materialSpecular;

    // Итоговый цвет света: сумма фонового, диффузного и зеркального освещения с учетом затухания и эффекта прожектора
    v_light_color = ambient + (diffuse + specular) * attenuation * spotlightEffect;

    // Повторная передача позиции источника света и камеры (для совместимости с фрагментным шейдером)
    v_lightPos = lightPos;
    v_viewPos = viewPos;
}
