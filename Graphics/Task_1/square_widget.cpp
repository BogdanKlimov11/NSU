// Подключение необходимых заголовков Qt для работы с OpenGL и рисованием
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QPainter>

#include "square_widget.h"

// Конструктор класса OpenGLWindow, принимает родительское окно (parent)
OpenGLWindow::OpenGLWindow(QWindow *parent) : QWindow(parent) {
    // Установка типа поверхности окна как OpenGL для поддержки рендеринга через OpenGL
    setSurfaceType(QWindow::OpenGLSurface);
}

// Деструктор класса, очищает выделенную память для m_device
OpenGLWindow::~OpenGLWindow() {
    delete m_device; // Удаление объекта устройства рисования
}

// Метод для отрисовки с использованием QPainter (пока пустой, можно переопределить)
void OpenGLWindow::render(QPainter *painter) {
    Q_UNUSED(painter); // Макрос для подавления предупреждений о неиспользуемом параметре
}

// Метод инициализации OpenGL (пока пустой, можно переопределить для настройки)
void OpenGLWindow::initialize() {
    // Здесь можно добавить код инициализации OpenGL
}

// Основной метод рендеринга с использованием OpenGL
void OpenGLWindow::render() {
    // Если устройство рисования еще не создано, создаем его
    if (!m_device)
        m_device = new QOpenGLPaintDevice;

    // Очистка буферов цвета, глубины и трафарета перед рисованием
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Установка размера устройства рисования с учетом плотности пикселей (для retina-дисплеев)
    m_device->setSize(size() * devicePixelRatio());
    m_device->setDevicePixelRatio(devicePixelRatio());

    // Создание объекта QPainter для рисования на устройстве OpenGL
    QPainter painter(m_device);
    render(&painter); // Вызов метода render с QPainter для кастомной отрисовки
}

// Метод для отложенного рендеринга, добавляет запрос на обновление в очередь событий
void OpenGLWindow::renderLater() {
    requestUpdate(); // Запрос обновления окна
}

// Переопределение метода обработки событий
bool OpenGLWindow::event(QEvent *event) {
    switch (event->type()) { // Проверка типа события
    case QEvent::UpdateRequest: // Если событие — запрос на обновление
        renderNow(); // Выполняем немедленную отрисовку
        return true; // Событие обработано
    default:
        return QWindow::event(event); // Передаем необработанные события базовому классу
    }
}

// Переопределение метода, вызываемого при отображении окна
void OpenGLWindow::exposeEvent(QExposeEvent *event) {
    Q_UNUSED(event); // Макрос для подавления предупреждений о неиспользуемом параметре

    // Если окно отображено на экране, выполняем отрисовку
    if (isExposed())
        renderNow();
}

// Метод для немедленной отрисовки с настройкой контекста OpenGL
void OpenGLWindow::renderNow() {
    // Если окно не отображено, прерываем выполнение
    if (!isExposed())
        return;

    bool needsInitialize = false; // Флаг, указывающий, нужна ли инициализация

    // Если OpenGL-контекст еще не создан, создаем его
    if (!m_context) {
        m_context = new QOpenGLContext(this); // Создание контекста для текущего окна
        m_context->setFormat(requestedFormat()); // Установка желаемого формата поверхности
        m_context->create(); // Инициализация контекста

        needsInitialize = true; // Указываем, что нужна инициализация
    }

    // Делаем контекст текущим для этого окна
    m_context->makeCurrent(this);

    // Если это первый запуск, инициализируем OpenGL-функции и настройки
    if (needsInitialize) {
        initializeOpenGLFunctions(); // Инициализация функций OpenGL из QOpenGLFunctions
        initialize(); // Вызов пользовательской инициализации
    }

    render(); // Выполнение отрисовки

    m_context->swapBuffers(this); // Обмен буферов для отображения результата на экране

    // Если анимация включена, запрашиваем следующий кадр
    if (m_animating)
        renderLater();
}

// Метод для включения или выключения анимации
void OpenGLWindow::setAnimating(bool animating) {
    m_animating = animating; // Установка значения флага анимации

    // Если анимация включена, запускаем отложенную отрисовку
    if (animating)
        renderLater();
}
