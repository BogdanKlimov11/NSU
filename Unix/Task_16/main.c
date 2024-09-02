#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <ctype.h>
#include <string.h>
#include <stdbool.h>

#define MAX_SIZE 256

int process(FILE *input) {
    char buffer[MAX_SIZE];
    size_t count = fread(buffer, sizeof(char), MAX_SIZE, input);
    fwrite(buffer, count, 1, stdout);
    if (ferror(input) == -1) {
        perror("fread(3)");
        if (pclose(input) == -1) {
            perror("pclose(1)");
        }
        return 1;
    }
    if (pclose(input) == -1) {
        perror("pclose(1)");
        return 1;
    }
    size_t i;
    for (i = 0; i < count; ++i) {
        buffer[i] = toupper(buffer[i]);
    }
    fwrite(buffer, count, 1, stdout);

    if (ferror(input) == -1) {
        perror("fwrite(3)");
        return 1;
    }
    return 0;
}

int main() {
    FILE *input = popen("echo hello world", "r");
    if (input == NULL) {
        perror("popen(2)");
        return 1;
    }
    return process(input);
}
