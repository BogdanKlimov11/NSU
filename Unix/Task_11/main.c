#include <stdio.h>
#include <ctype.h>
#include <termios.h>
#include <unistd.h>

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
    printf("(Y)es or (N)o?\n");
    while (1) {
        char buf;
        if (read(STDIN_FILENO, &buf, 1) == 1) {
            if (tolower(buf) == 'y') {
                printf("Yes\n");
                break;
            } else if (tolower(buf) == 'n') {
                printf("No\n");
                break;
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
