#include <stdio.h>
#include <ctype.h>
#include <termios.h>
#include <unistd.h>

#define STR_SIZE 40

typedef struct {
    char data[STR_SIZE];
    size_t size;
} string;

void string_verase(string *str, size_t count) {
    str->size -= count;
    for (size_t i = 0; i < count; ++i)
        write(STDOUT_FILENO, "\b \b", 3);
}

void string_newline(string *str) {
    write(STDOUT_FILENO, "\n", 1);
    str->size = 0;
}

void string_vwerase(string *str) {
    size_t count = 0;
    for (count = str->size - 1; count >= 0; --count) /*spaces*/
        if (isspace(str->data[count]))
            break;

    for (; count >= 0; --count) /*word*/
        if (!isspace(str->data[count]))
            break;

    string_verase(str, str->size - count - 1);
}

void string_wrap(string *str) {
    size_t count, old_size = str->size;
    for (count = old_size - 1; count >= 0; --count)
        if (isspace(str->data[count]))
            break;

    if (count >= 0) {
        string_verase(str, str->size - ++count);
        string_newline(str);
        /* перевел строку, обнулил size, но в data еще лежит
         * последнее слово - переписываем его в новую строку*/
        while (count++ < old_size) {
            write(STDOUT_FILENO, &(str->data[count]), 1);
            str->data[str->size++] = str->data[count];
        }
    } else { /*слово >40 букв, придется делить*/
        string_newline(str);
    }
}

int main() {
    if (isatty(STDIN_FILENO) == 0) {
        fprintf(stderr, "Standard input is not a terminal");
        return -1;
    }
    struct termios attributes;
    struct termios old_attributes;
    if (tcgetattr(STDIN_FILENO, &attributes) == -1) {
        perror("Couldn't get terminal attributes");
        return -1;
    }
    old_attributes = attributes;
    attributes.c_lflag &= ~(ICANON | ECHO);
    attributes.c_cc[VMIN] = 1;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &attributes) == -1) {
        perror("Couldn't set terminal attributes");
        return -1;
    }

    string str;
    str.size = 0;
    while (1) {
        char buf;
        if (read(STDIN_FILENO, &buf, 1) == 1) {
            if (isprint(buf)) {
                if (str.size == STR_SIZE) {
                    if (isspace(buf)) {
                        string_newline(&str);
                    } else {
                        string_wrap(&str);
                    }
                }
                write(STDOUT_FILENO, &buf, 1);
                str.data[str.size++] = buf;
            } else if ((attributes.c_cc[VERASE] == buf) && (str.size)) {
                string_verase(&str, 1);
            } else if (attributes.c_cc[VKILL] == buf) {
                string_verase(&str, str.size);
            } else if (attributes.c_cc[VWERASE] == buf) {
                string_vwerase(&str);
            } else if (attributes.c_cc[VEOF] == buf && !str.size) {
                break;
            } else if ('\n' == buf) {
                string_newline(&str);
            } else {
                write(STDOUT_FILENO, "\a", 1);
            }
        } else {
            perror("Couldn't read from standard input");
        }
    }

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_attributes) == -1) {
        perror("Couldn't restore terminal attributes");
        return -1;
    }
    return 0;
}
