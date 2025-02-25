#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ALPHABET_SIZE 256  // Размер алфавита (для ASCII)

// Структура для хранения состояния автомата Ахо-Корасика
typedef struct AhoCorasick {
    int *fail;              // Массив функций неудачи
    int **edges;            // Массив ребер
    int *output;            // Массив выходных состояний (множество найденных паттернов)
    int num_nodes;          // Количество вершин
} AhoCorasick;

// Функция для создания нового автомата Ахо-Корасика
AhoCorasick* create_ac_automaton() {
    AhoCorasick *ac = (AhoCorasick *)malloc(sizeof(AhoCorasick));
    ac->num_nodes = 1;

    // Выделяем память для структур
    ac->edges = (int **)malloc(ALPHABET_SIZE * sizeof(int *));
    for (int i = 0; i < ALPHABET_SIZE; i++) {
        ac->edges[i] = (int *)malloc(ALPHABET_SIZE * sizeof(int));
    }

    ac->fail = (int *)malloc(ALPHABET_SIZE * sizeof(int));
    ac->output = (int *)malloc(ALPHABET_SIZE * sizeof(int));
    return ac;
}

// Функция для добавления паттерна в автомат
void add_pattern(AhoCorasick *ac, const char *pattern) {
    int current_node = 0;
    for (int i = 0; pattern[i] != '\0'; i++) {
        unsigned char c = pattern[i];
        if (ac->edges[current_node][c] == -1) {
            ac->edges[current_node][c] = ac->num_nodes++;
        }
        current_node = ac->edges[current_node][c];
    }
    ac->output[current_node] = 1;  // Маркируем конец паттерна
}

// Функция для построения функций неудачи (BFS)
void build_failure(AhoCorasick *ac) {
    ac->fail[0] = 0;  // Для корня функция неудачи равна корню

    // Очередь для BFS
    int *queue = (int *)malloc(ac->num_nodes * sizeof(int));
    int front = 0, rear = 0;

    // Строим уровни для первого уровня
    for (int c = 0; c < ALPHABET_SIZE; c++) {
        if (ac->edges[0][c] != -1) {
            ac->fail[ac->edges[0][c]] = 0;
            queue[rear++] = ac->edges[0][c];
        }
    }

    // BFS для построения функции неудачи
    while (front < rear) {
        int current_node = queue[front++];
        for (int c = 0; c < ALPHABET_SIZE; c++) {
            if (ac->edges[current_node][c] != -1) {
                int fail_state = ac->fail[current_node];
                while (ac->edges[fail_state][c] == -1) {
                    fail_state = ac->fail[fail_state];
                }
                ac->fail[ac->edges[current_node][c]] = ac->edges[fail_state][c];
                queue[rear++] = ac->edges[current_node][c];
            }
        }
    }

    free(queue);
}

// Функция для поиска подстрок в строке
void search(AhoCorasick *ac, const char *text) {
    int current_node = 0;

    for (int i = 0; text[i] != '\0'; i++) {
        unsigned char c = text[i];

        // Следуем по дереву, если ребра не найдены, то идем по функции неудачи
        while (ac->edges[current_node][c] == -1) {
            current_node = ac->fail[current_node];
        }
        current_node = ac->edges[current_node][c];

        // Проверка на выходные состояния
        if (ac->output[current_node]) {
            printf("Подстрока найдена на позиции %d\n", i);
        }
    }
}

// Функция для инициализации и выполнения поиска
void substring_search(const char *text, const char *pattern) {
    AhoCorasick *ac = create_ac_automaton();

    // Добавляем паттерн
    add_pattern(ac, pattern);
    // Строим функции неудачи
    build_failure(ac);

    // Выполняем поиск в строке
    search(ac, text);

    // Освобождаем память
    free(ac->fail);
    free(ac->output);
    for (int i = 0; i < ALPHABET_SIZE; i++) {
        free(ac->edges[i]);
    }
    free(ac->edges);
    free(ac);
}

int main() {
    const char *text = "stog siena igolka v stoge";
    const char *pattern = "stog";
    
    substring_search(text, pattern);  // Поиск одной подстроки

    return 0;
}
