#include <stdio.h>
#include <ctype.h>

void strclear(char *string) {
    int i = 0, j = 0;
    int space_found = 0;  // Флаг, чтобы отслеживать, если был пробел подряд

    // Перебираем строку
    while (string[i] != '\0') {
        if (isspace(string[i])) {
            if (!space_found) {  // Если пробела еще не было, то ставим один
                string[j++] = ' ';
                space_found = 1;
            }
        } else {
            string[j++] = string[i];  // Просто копируем символ
            space_found = 0;  // Сброс флага
        }
        i++;
    }

    string[j] = '\0';  // Завершаем строку
}

int main() {
    char buf[] = "ab    cd";
    printf("<%s>\n", buf);  // <ab    cd>
    strclear(buf);
    printf("<%s>\n", buf);  // <ab cd>
    return 0;
}
