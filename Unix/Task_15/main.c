#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <ctype.h>
#include <string.h>

#define MAX_SIZE 256

void closep(int pipes[2]) {
    if (close(pipes[0]) == -1) {
        printf("Failed to close pipes[0]\n");
    }
    if (close(pipes[1]) == -1) {
        printf("Failed to close pipes[1]\n");
    }
}

int main() {
    char buffer[MAX_SIZE] = "hello world";
    int size = 0;
    int pipes[2];
    if (pipe(pipes) == -1) {
        perror("pipe()");
        return 1;
    }
    pid_t child = fork();
    switch (child) {
        case -1: {
            perror("fork()");
            closep(pipes);
            return 1;
        }
        case 0: {
            while ((size = read(pipes[0], buffer, MAX_SIZE)) == -1) {
                if (errno != EINTR) {
                    perror("read()");
                    return 1;
                }
            }
            if (close(pipes[0]) == -1) {
                printf("Failed to close pipes[0]\n");
            }
            size_t i;
            for (i = 0; i < size; ++i)
                buffer[i] = toupper(buffer[i]);
            printf("%s\n", buffer);
            return 0;
        }
        default: {
            if (write(pipes[1], buffer, strlen(buffer) + 1) == -1) {
                perror("write()");
                closep(pipes);
                return 1;
            }
            if (close(pipes[1]) == -1) {
                printf("Failed to close pipes[1]\n");
            }
            printf("%s\n", buffer);
            int status;
            pid_t ChildPid;
            do {
                ChildPid = waitpid(child, &status, 0);
                if (ChildPid == -1) {
                    perror("waitpid() error");
                    return 1;
                }
                if (WIFEXITED(status)) {
                    printf("Exit code %d\n", WEXITSTATUS(status));
                } else if (WIFSIGNALED(status)) {
                    printf("Closed by signal: %d\n", WTERMSIG(status));
                } else if (WIFSTOPPED(status)) {
                    printf("Stopped by signal: %d\n", WSTOPSIG(status));
                }
            } while (!WIFEXITED(status) && !WIFSIGNALED(status));
            break;
        }
    }
    return 0;
}
