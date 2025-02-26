#ifndef SQUARE_WIDGET_H
#define SQUARE_WIDGET_H

// Подключение необходимых заголовочных файлов из Qt для работы с окнами и OpenGL
#include <QWindow>
#include <QOpenGLFunctions>

// Предварительное объявление классов Qt, которые будут использоваться,
// чтобы избежать необходимости включения их заголовков здесь
QT_FORWARD_DECLARE_CLASS(QPainter)
QT_FORWARD_DECLARE_CLASS(QOpenGLContext)
QT_FORWARD_DECLARE_CLASS(QOpenGLPaintDevice)

// Определение класса OpenGLWindow, который наследуется от QWindow и QOpenGLFunctions
class OpenGLWindow : public QWindow, protected QOpenGLFunctions {
    Q_OBJECT // Макрос Qt для поддержки сигналов и слотов

public:
    // Конструктор класса с необязательным родительским окном (по умолчанию nullptr)
    explicit OpenGLWindow(QWindow *parent = nullptr);

    // Деструктор класса для очистки ресурсов
    ~OpenGLWindow();

    // Виртуальная функция для отрисовки с использованием QPainter
    virtual void render(QPainter *painter);

    // Виртуальная функция для отрисовки с использованием OpenGL
    virtual void render();

    // Виртуальная функция для инициализации OpenGL-контекста и ресурсов
    virtual void initialize();

    // Метод для включения или выключения анимации
    void setAnimating(bool animating);

public slots: // Секция слотов (методов, которые могут вызываться через сигналы)
    // Слот для отложенной отрисовки (добавляет запрос на рендеринг в очередь)
    void renderLater();

    // Слот для немедленной отрисовки
    void renderNow();

protected:
    // Переопределение метода обработки событий от базового класса QWindow
    bool event(QEvent *event) override;

    // Переопределение метода, вызываемого при отображении окна на экране
    void exposeEvent(QExposeEvent *event) override;

private:
    // Приватная переменная, указывающая, включена ли анимация (по умолчанию false)
    bool m_animating = false;

    // Указатель на OpenGL-контекст, используемый для работы с OpenGL
    QOpenGLContext *m_context = nullptr;

    // Указатель на устройство для рисования с помощью OpenGL
    QOpenGLPaintDevice *m_device = nullptr;
};

#endif // SQUARE_WIDGET_H
