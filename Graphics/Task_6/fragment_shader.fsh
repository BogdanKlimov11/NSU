// Вариационные переменные, переданные из вершинного шейдера
varying vec3 v_norm; // Нормаль вершины
varying vec2 v_text_coord; // Текстурные координаты (UV)
varying vec3 v_tangent; // Касательный вектор
varying vec3 v_bitangent; // Бинормальный вектор

// Униформные переменные
uniform sampler2D day_texture; // Текстура дневной поверхности
uniform sampler2D normal_map; // Карта нормалей
uniform vec3 viewPos; // Позиция камеры
uniform vec3 lightPos; // Позиция источника света

// Главная функция фрагментного шейдера
void main() {
    // Загрузка цвета из дневной текстуры
    vec3 albedo = texture2D(day_texture, v_text_coord).rgb;
    // Загрузка значения из карты нормалей
    vec3 normalMap = texture2D(normal_map, v_text_coord).rgb;

    // Нормализация карты нормалей из диапазона [0,1] в [-1,1]
    normalMap = normalize(normalMap * 2.0 - 1.0);

    // Построение TBN-матрицы (Tangent-Bitangent-Normal)
    vec3 N = normalize(v_norm); // Нормализованная нормаль
    vec3 T = normalize(v_tangent); // Нормализованный касательный вектор
    vec3 B = normalize(v_bitangent); // Нормализованный бинормальный вектор
    mat3 TBN = mat3(T, B, N); // Матрица перехода в пространство касательных
    vec3 normal = normalize(TBN * normalMap); // Нормаль с учетом карты нормалей

    // Вычисление направлений света и взгляда
    vec3 lightDir = normalize(lightPos); // Направление света
    vec3 viewDir = normalize(viewPos); // Направление взгляда (к камере)

    // Диффузное освещение
    float diff = max(dot(normal, lightDir), 0.0); // Диффузный коэффициент
    vec3 diffuse = diff * albedo; // Диффузная компонента

    // Зеркальное освещение (модель Фонга)
    vec3 reflectDir = reflect(-lightDir, normal); // Направление отражения света
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0); // Зеркальный коэффициент
    vec3 specular = spec * vec3(0.5); // Зеркальная компонента (белый блик, интенсивность 0.5)

    // Фоновое (амбиентное) освещение
    vec3 ambient = 0.05 * albedo; // 5% от цвета текстуры

    // Итоговый цвет фрагмента (фон + диффузия + блик, полная непрозрачность)
    gl_FragColor = vec4(ambient + diffuse + specular, 1.0);
}
