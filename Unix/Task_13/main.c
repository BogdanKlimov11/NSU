#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <stdlib.h>

struct termios old_attributes, attributes;
int counter;

void sigcatch(int sig) {
    if (sig == SIGQUIT) {
        printf("Count of clicks DEL: %d\n", counter);
        tcsetattr(fileno(stdin), TCSANOW, &old_attributes);
        exit(0);
    }
    printf("\a");
    fflush(stdout);
    ++counter;
}

int main() {
    tcgetattr(fileno(stdin), &old_attributes);
    attributes = old_attributes;
    attributes.c_lflag &= ~ECHO;
    attributes.c_cc[VINTR] = 27;   /* control characters: DEL */
    tcsetattr(fileno(stdin), TCSANOW, &attributes);
    while (1) {
        signal(SIGINT, sigcatch);
        signal(SIGQUIT, sigcatch);
    }
    return 0;
}
