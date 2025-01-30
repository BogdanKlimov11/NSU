#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 1024  // Максимальная длина строки

void compare_files(const char *file1, const char *file2) {
    FILE *fp1 = fopen(file1, "r");
    FILE *fp2 = fopen(file2, "r");

    if (!fp1 || !fp2) {
        printf("Ошибка: невозможно открыть один из файлов.\n");
        return;
    }

    char line1[MAX_LINE], line2[MAX_LINE];
    int line_num = 0;

    while (fgets(line1, MAX_LINE, fp1) && fgets(line2, MAX_LINE, fp2)) {
        line_num++;
        if (strcmp(line1, line2) != 0) {
            printf("Различие в строке %d:\n", line_num);
            printf("Файл 1: %sФайл 2: %s", line1, line2);
            fclose(fp1);
            fclose(fp2);
            return;
        }
    }

    // Проверка, если один файл длиннее другого
    if (fgets(line1, MAX_LINE, fp1)) {
        printf("Файл 1 длиннее, отличие на строке %d\n", ++line_num);
    } else if (fgets(line2, MAX_LINE, fp2)) {
        printf("Файл 2 длиннее, отличие на строке %d\n", ++line_num);
    } else {
        printf("Файлы идентичны.\n");
    }

    fclose(fp1);
    fclose(fp2);
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Использование: %s <файл1> <файл2>\n", argv[0]);
        return 1;
    }

    compare_files(argv[1], argv[2]);
    return 0;
}
