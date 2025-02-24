#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_WORD_LENGTH 100
#define MAX_SENTENCE_LENGTH 10
#define MAX_TEXT_LENGTH 10000

// Функция для загрузки текста из файла в массив слов
int load_text(const char *filename, char words[MAX_TEXT_LENGTH][MAX_WORD_LENGTH]) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Ошибка при открытии файла.\n");
        return 0;
    }
    
    int word_count = 0;
    while (fscanf(file, "%s", words[word_count]) == 1) {
        word_count++;
    }
    
    fclose(file);
    return word_count;
}

// Функция для генерации случайного предложения
void generate_sentence(char words[MAX_TEXT_LENGTH][MAX_WORD_LENGTH], int word_count, char *sentence) {
    int sentence_length = rand() % (MAX_SENTENCE_LENGTH - 3) + 3;
    int word_indices[MAX_SENTENCE_LENGTH];
    for (int i = 0; i < sentence_length; i++) {
        word_indices[i] = rand() % word_count;
    }

    // Составляем предложение
    sentence[0] = toupper(words[word_indices[0]][0]);
    strcat(sentence, words[word_indices[0]] + 1);

    for (int i = 1; i < sentence_length; i++) {
        strcat(sentence, " ");
        strcat(sentence, words[word_indices[i]]);
    }

    // Завершаем предложение точкой или восклицательным знаком
    strcat(sentence, rand() % 2 ? "." : "!");
}

// Функция для проверки ввода и подсчета статистики
void check_typing_speed(const char *training_text) {
    printf("Тренировочный текст:\n%s\n", training_text);
    printf("\nНачните вводить этот текст. Нажмите Enter для начала.");
    getchar();  // Ожидаем нажатия Enter
    
    clock_t start_time = clock();
    char typed_text[MAX_TEXT_LENGTH];
    printf("Введите текст: ");
    fgets(typed_text, MAX_TEXT_LENGTH, stdin);
    clock_t end_time = clock();
    
    // Подсчитываем ошибки
    int errors = 0;
    for (int i = 0; i < strlen(training_text) && i < strlen(typed_text); i++) {
        if (training_text[i] != typed_text[i]) {
            errors++;
        }
    }
    
    double time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    double typing_speed_chars = strlen(typed_text) / time_taken * 60;
    double typing_speed_words = (strlen(typed_text) / 5) / time_taken * 60;  // Среднее количество символов в слове - 5
    
    // Выводим статистику
    printf("\nСтатистика:\n");
    printf("Ошибок: %d\n", errors);
    printf("Время: %.2f секунд\n", time_taken);
    printf("Скорость ввода: %.2f символов в минуту\n", typing_speed_chars);
    printf("Скорость ввода: %.2f слов в минуту\n", typing_speed_words);
}

int main() {
    srand(time(NULL));
    
    // Загрузка текста из файла
    char words[MAX_TEXT_LENGTH][MAX_WORD_LENGTH];
    int word_count = load_text("text.txt", words);
    if (word_count == 0) return 1;

    // Генерация тренировочного текста
    char training_text[MAX_TEXT_LENGTH] = "";
    int sentence_count = rand() % 4 + 5;  // 5-8 предложений
    for (int i = 0; i < sentence_count; i++) {
        char sentence[MAX_TEXT_LENGTH] = "";
        generate_sentence(words, word_count, sentence);
        strcat(training_text, sentence);
        strcat(training_text, " ");
    }
    
    // Проверка ввода пользователя
    check_typing_speed(training_text);
    
    return 0;
}
