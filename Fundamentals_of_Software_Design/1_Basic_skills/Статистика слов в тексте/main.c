#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define MAX_WORD_LENGTH 100
#define TABLE_SIZE 1000

// Структура для хранения слова и его количества
typedef struct WordNode {
    char word[MAX_WORD_LENGTH];
    int count;
    struct WordNode* next;
} WordNode;

// Хэш-таблица
WordNode* hash_table[TABLE_SIZE];

// Функция для вычисления хэш-значения
unsigned int hash(const char *word) {
    unsigned int hash_value = 0;
    while (*word) {
        hash_value = hash_value * 31 + tolower(*word);
        word++;
    }
    return hash_value % TABLE_SIZE;
}

// Функция для добавления слова в хэш-таблицу
void add_word(const char *word) {
    unsigned int index = hash(word);
    WordNode *node = hash_table[index];
    
    // Поиск, есть ли уже это слово в таблице
    while (node != NULL) {
        if (strcmp(node->word, word) == 0) {
            node->count++;
            return;
        }
        node = node->next;
    }
    
    // Если слово не найдено, создаем новый узел
    WordNode *new_node = (WordNode *)malloc(sizeof(WordNode));
    strcpy(new_node->word, word);
    new_node->count = 1;
    new_node->next = hash_table[index];
    hash_table[index] = new_node;
}

// Функция для обработки текста
void process_text(FILE *file) {
    char word[MAX_WORD_LENGTH];
    int i = 0;
    
    while (fscanf(file, "%s", word) == 1) {
        // Убираем знаки пунктуации с начала и конца слова
        i = 0;
        for (int j = 0; word[j] != '\0'; j++) {
            if (isalnum(word[j])) {
                word[i++] = tolower(word[j]);
            }
        }
        word[i] = '\0';
        
        if (i > 0) {
            add_word(word);
        }
    }
}

// Функция для записи статистики в файл
void save_statistics(FILE *outfile) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        WordNode *node = hash_table[i];
        while (node != NULL) {
            fprintf(outfile, "%s %d\n", node->word, node->count);
            node = node->next;
        }
    }
}

// Основная функция
int main() {
    FILE *infile = fopen("infile.txt", "r");
    if (infile == NULL) {
        printf("Ошибка при открытии файла.\n");
        return 1;
    }

    FILE *outfile = fopen("outfile.txt", "w");
    if (outfile == NULL) {
        printf("Ошибка при создании файла для записи.\n");
        fclose(infile);
        return 1;
    }

    // Обработка текста
    process_text(infile);
    
    // Запись статистики
    save_statistics(outfile);
    
    // Закрытие файлов
    fclose(infile);
    fclose(outfile);
    
    printf("Статистика записана в файл: outfile.txt\n");
    return 0;
}
