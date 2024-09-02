#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

void printStat(pid_t pid, int stat)
{
    printf("Process status - %d\n", stat);
    if(WCOREDUMP(stat))
        printf("Process %d ended with core dump %d\n", pid, WCOREDUMP(stat));
    if(WEXITSTATUS(stat))
        printf("Process %d ended with exit code %d\n", pid, WEXITSTATUS(stat));
    if(WIFCONTINUED(stat))
        printf("Process %d ended with continued %d\n", pid, WIFCONTINUED(stat));
    if(WIFEXITED(stat))
        printf("Process %d terminated normally with %d\n", pid, WIFEXITED(stat));
    if(WIFSIGNALED(stat))
        printf("Process %d terminated due to signal %d\n", pid, WIFSIGNALED(stat));
    if(WIFSTOPPED(stat))
        printf("Process %d currently stopped\n", pid);
    if(WSTOPSIG(stat))
        printf("Process %d stopped due to signal %d\n", pid, WSTOPSIG(stat));
    if(WTERMSIG(stat))
        printf("Process %d terminated due to signal %d\n", pid, WTERMSIG(stat));
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("usage: ./prog [cmd] [arg]\n");
        return 1;
    }

    pid_t pid;
    pid = fork();

    switch (pid) {
        case -1:
            perror("Fork failure\n");
            return 1;
        case 0:
            execvp(argv[1], argv + 1);
            perror("Execute argv error\n");
            return 1;
        default:{
            int childStat;
            if(wait(&childStat) == -1)
            {
                perror("Wait failure\n");
                return 1;
            }
            printStat(pid, childStat);
            break;
        }
    }

    return 0;
}
