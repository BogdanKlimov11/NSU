#include <QDebug> // Для вывода отладочной информации
#include <QImage> // Для загрузки и работы с изображениями

#include "earth_material.h" // Подключение заголовочного файла класса MaterialEarth

// Конструктор класса MaterialEarth
MaterialEarth::MaterialEarth() {
    // Загрузка дневной текстуры Земли
    QImage dayImage(":/earth_map.png");
    if (dayImage.isNull()) {
        // Вывод сообщения об ошибке, если изображение не удалось загрузить
        qDebug() << "Failed to load day texture: earth_map.png";
    } else {
        // Создание текстуры OpenGL из изображения (зеркалирование для корректной ориентации)
        day_texture_ = std::make_shared<QOpenGLTexture>(dayImage.mirrored());
        // Установка фильтра для уменьшения текстуры (линейная интерполяция с мип-маппингом)
        day_texture_->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        // Установка фильтра для увеличения текстуры (линейная интерполяция)
        day_texture_->setMagnificationFilter(QOpenGLTexture::Linear);
        // Установка режима повторения текстуры по краям
        day_texture_->setWrapMode(QOpenGLTexture::Repeat);
        // Генерация мип-уровней для текстуры
        day_texture_->generateMipMaps();
        // Вывод сообщения об успешной загрузке
        qDebug() << "Day texture loaded: earth_map.png";
    }

    // Загрузка карты нормалей
    QImage normalMapImage(":/normal_map.png");
    if (normalMapImage.isNull()) {
        // Вывод сообщения об ошибке, если карта нормалей не загрузилась
        qDebug() << "Failed to load normal map: normal_map.png";
    } else {
        // Создание текстуры OpenGL из карты нормалей (зеркалирование для корректной ориентации)
        normal_map_ = std::make_shared<QOpenGLTexture>(normalMapImage.mirrored());
        // Установка фильтра для уменьшения текстуры
        normal_map_->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        // Установка фильтра для увеличения текстуры
        normal_map_->setMagnificationFilter(QOpenGLTexture::Linear);
        // Установка режима повторения текстуры по краям
        normal_map_->setWrapMode(QOpenGLTexture::Repeat);
        // Генерация мип-уровней для карты нормалей
        normal_map_->generateMipMaps();
        // Вывод сообщения об успешной загрузке
        qDebug() << "Normal map loaded: normal_map.png";
    }
}

// Метод для привязки текстур к шейдерной программе
void MaterialEarth::bind(QOpenGLShaderProgram* program) {
    // Привязка дневной текстуры, если она создана
    if (day_texture_ && day_texture_->isCreated()) {
        day_texture_->bind(0); // Привязка к текстурному блоку 0
        program->setUniformValue("day_texture", 0); // Установка униформы для текстуры
        qDebug() << "Day texture bound";
    }
    // Привязка карты нормалей, если она создана
    if (normal_map_ && normal_map_->isCreated()) {
        normal_map_->bind(2); // Привязка к текстурному блоку 2
        program->setUniformValue("normal_map", 2); // Установка униформы для карты нормалей
        qDebug() << "Normal map bound";
    }
}
