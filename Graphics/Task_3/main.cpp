#include <QApplication> // Подключение основного класса Qt для управления приложением
#include <QSurfaceFormat> // Подключение класса для настройки формата поверхности OpenGL

#include "light_widget.h" // Подключение заголовочного файла пользовательского виджета управления освещением
#include "square_window.h" // Подключение заголовочного файла пользовательского окна рендеринга

// Главная функция программы
int main(int argc, char **argv) {
    // Создание объекта приложения Qt, передача аргументов командной строки
    QApplication app(argc, argv);

    // Создание объекта для настройки формата поверхности OpenGL
    QSurfaceFormat format;
    // Установка количества сэмплов для мультисэмплинга (для сглаживания)
    format.setSamples(16);
    // Установка версии OpenGL (2.1)
    format.setVersion(2, 1);
    // Установка формата по умолчанию для всех поверхностей OpenGL в приложении
    QSurfaceFormat::setDefaultFormat(format);

    // Создание объекта окна рендеринга (SquareWindow)
    SquareWindow *renderWindow = new SquareWindow();
    // Установка заголовка окна рендеринга
    renderWindow->setWindowTitle("Дисплей");
    // Установка размера окна рендеринга (800x600 пикселей)
    renderWindow->resize(800, 600);
    // Установка иконки окна рендеринга из ресурса
    renderWindow->setWindowIcon(QIcon(":/display.ico"));
    // Отображение окна рендеринга на экране
    renderWindow->show();

    // Создание объекта виджета управления освещением (LightWidget), связанного с окном рендеринга
    LightWidget *controlWindow = new LightWidget(renderWindow);
    // Установка заголовка окна управления
    controlWindow->setWindowTitle("Панель управления");
    // Установка размера окна управления (400x600 пикселей)
    controlWindow->resize(400, 600);
    // Установка иконки окна управления из ресурса
    controlWindow->setWindowIcon(QIcon(":/control.ico"));
    // Отображение окна управления на экране
    controlWindow->show();

    // Перемещение окна рендеринга в позицию (100, 100) на экране
    renderWindow->move(100, 100);
    // Перемещение окна управления в позицию (900, 100) на экране
    controlWindow->move(900, 100);

    // Запуск основного цикла событий приложения и возврат кода завершения
    return app.exec();
}
