#include <unistd.h>
#include <stdio.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <memory.h>

#define MEM_SIZE 8

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
    int pr = 0; /*read pointer*/
    while (1) {
        if (semop(semid, &wait_read, 1) == -1) {
            perror("semop");
            return 1;
        }
        //sleep(1); /*for demonstrate*/
        printf("%c", *(shm_ptr+pr));
        fflush(stdout);
        if (*(shm_ptr+pr) == '#') break;
        pr = (pr + 1) % MEM_SIZE;
        if (semop(semid, &notify_write, 1) == -1) {
            perror("semop");
            return 1;
        }
    }
    if (shmdt(shm_ptr) == -1) {
        perror("shmdt");
        return 1;
    }
    return 0;
}