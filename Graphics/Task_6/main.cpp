#include <QApplication> // Для работы с GUI-приложением Qt

#include "main_window.h" // Главное окно приложения

int main(int argc, char *argv[]) {
    QApplication a(argc, argv); // Инициализация приложения
    MainWindow w; // Создание главного окна
    w.show(); // Отображение окна
    return a.exec(); // Запуск цикла обработки событий
}
