#include <stdio.h>
#include <ctype.h>

void count_file_stats(const char *filename, int *char_count, int *line_count, int *word_count) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Ошибка: файл '%s' не найден.\n", filename);
        return;
    }

    int ch, prev = ' ';
    *char_count = *line_count = *word_count = 0;

    while ((ch = fgetc(file)) != EOF) {
        (*char_count)++;
        if (ch == '\n') (*line_count)++;
        if (isspace(ch) && !isspace(prev)) (*word_count)++;
        prev = ch;
    }

    fclose(file);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Использование: %s <имя_файла>\n", argv[0]);
        return 1;
    }

    int chars, lines, words;
    count_file_stats(argv[1], &chars, &lines, &words);

    printf("Символов: %d\n", chars);
    printf("Строк: %d\n", lines);
    printf("Слов: %d\n", words);

    return 0;
}
