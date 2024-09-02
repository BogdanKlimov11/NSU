#include <unistd.h>
#include <stdio.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <memory.h>

#define MEM_SIZE 64

int main() {
    int semid, shmid;
    struct sembuf wait_read = {0, -1, 0};
    struct sembuf notify_write = {1, 1, 0};
    if ((semid = semget(getuid(), 2, 0)) == -1) {
        perror("semget");
        return 1;
    }
    if ((shmid = shmget(getuid(), MEM_SIZE, 0)) == -1) {
        perror("shmget");
        return 1;
    }
    char *shm_ptr = shmat(shmid, NULL, 0);
    while (strcmp(shm_ptr, "bye\n") != 0) {
        if (semop(semid, &wait_read, 1) == -1) {
            perror("semop");
            return 1;
        }
        sleep(2); /*for demonstrate*/
        printf("%s", shm_ptr);
        semop(semid, &notify_write, 1);
    }
    if (shmdt(shm_ptr) == -1) {
        perror("shmdt");
        return 1;
    }
    return 0;
}