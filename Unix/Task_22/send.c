#include <unistd.h>
#include <stdio.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <memory.h>

#define MEM_SIZE 64

int main() {
    int semid, shmid;
    struct sembuf wait_write = {1, -1, 0};
    struct sembuf notify_read = {0, 1, 0};
    if ((semid = semget(getuid(), 2, IPC_CREAT | 0666)) == -1) {
        perror("semget");
        return 1;
    }
    if ((shmid = shmget(getuid(), MEM_SIZE, IPC_CREAT | 0666)) == -1) {
        perror("shmget");
        return 1;
    }
    char *shm_ptr = shmat(shmid, NULL, 0);
    char buf[MEM_SIZE];
    if (semctl(semid, 1, SETVAL, 1) == -1) {
        perror("semctl");
        return 1;
    }
    while (strcmp(shm_ptr, "bye\n") != 0) {
        fgets(buf, MEM_SIZE, stdin);
        if (semop(semid, &wait_write, 1) == -1) {
            perror("semop");
            return 1;
        }
        sprintf(shm_ptr, "%s", buf);
        semop(semid, &notify_read, 1);
    }
    if (shmdt(shm_ptr) == -1) {
        perror("shmdt");
        return 1;
    }
    if (semctl(semid, 0, IPC_RMID, 0) == -1) {
        perror("semctl");
        return 1;
    }
    if (shmctl(shmid, IPC_RMID, 0) == -1) {
        perror("semctl");
        return 1;
    }
    return 0;
}