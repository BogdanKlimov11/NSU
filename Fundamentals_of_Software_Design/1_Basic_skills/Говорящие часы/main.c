#include <stdio.h>
#include <stdlib.h>
#include <time.h>

const char* numbers[] = {
    "ноль", "один", "два", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять",
    "десять", "одиннадцать", "двенадцать", "тринадцать", "четырнадцать", "пятнадцать", "шестнадцать",
    "семнадцать", "восемнадцать", "девятнадцать", "двадцать", "тридцать", "сорок", "пятьдесят", "шестидесяти"
};

// Функция для перевода числа в текст
const char* number_to_text(int num) {
    if (num < 20) return numbers[num];
    else if (num < 30) return "двадцать";
    else if (num < 40) return "тридцать";
    else if (num < 50) return "сорок";
    else return "пятьдесят";
}

// Функция для вывода времени
void time_to_text(int hour, int minute) {
    if (minute == 0) {
        printf("%s часов ровно\n", number_to_text(hour));
    } else {
        printf("%s часов %s минут\n", number_to_text(hour), number_to_text(minute));
    }
}

// Функция для вывода дня недели и даты
void date_to_text() {
    const char* days[] = {"понедельник", "вторник", "среда", "четверг", "пятница", "суббота", "воскресенье"};
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    
    printf("%s, %d %s %d\n", days[tm_info->tm_wday], tm_info->tm_mday, 
           "январь февраля март апрель май июнь июль август сентябрь октябрь ноябрь декабрь"[tm_info->tm_mon * 7],
           tm_info->tm_year + 1900);
}

int main() {
    time_t t;
    struct tm *tm_info;
    
    // Получаем текущее время
    t = time(NULL);
    tm_info = localtime(&t);
    
    int hour = tm_info->tm_hour;
    int minute = tm_info->tm_min;
    
    // Выводим текущую дату и время
    date_to_text();
    time_to_text(hour, minute);

    return 0;
}
