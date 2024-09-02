#include <stdio.h>
#include <sys/sem.h>
#include <unistd.h>
#include <errno.h>

#define semPartA 0
#define semPartB 1
#define semPartC 2
#define semVidget 3

int main() {
    int semid = semget(getuid(), 4, IPC_CREAT | 0660);
    if (semid == -1) {
        perror("semget");
        return 1;
    }
    for (int i = 1; i<=5; ++i) {
        struct sembuf semops[2] = {{semPartC, -1, 0}, {semVidget, -1, 0}}; //num, op, flg
        if (semop(semid, semops, 2) == -1) {
            perror("semop");
            return 1;
        }
        printf("Create: Widget %d\n", i);
    }
    if (semctl(semid, 0, IPC_RMID, 0) == -1) {
        perror("semctl");
        return 1;
    }
    return 0;
}