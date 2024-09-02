#include <stdio.h> // perror
#include <unistd.h> // fork
#include <sys/wait.h>

int main(int argc, char *argv[]) {
    pid_t pid;
    printf("pid: %d\n", getpid());

    pid = fork();

    switch (pid) {
        case -1:
            perror("Fork fail\n");
            return -1;
        case 0:
            printf("pid of a child: %d\n", getpid());
            execlp("cat", "cat", argv[1], NULL);
            perror("cat\n");
        default: {
            int child_stat;
            printf("Parent begin\n");
            wait(&child_stat);
            printf("Parent end\n");
        }

    }

    return 0;
}
