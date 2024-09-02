#include <unistd.h>
#include <stdio.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <memory.h>

#define MEM_SIZE 8

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
    if (semctl(semid, 1, SETVAL, MEM_SIZE) == -1) { /*Семафор на запись init MEM_SIZE*/
        perror("semctl");
        return 1;
    }
    char *shm_ptr = shmat(shmid, NULL, 0);
    char buf[MEM_SIZE];
    int pw = 0; /*write pointer*/
    int ENDFLAG = 1;
    while (ENDFLAG) {
        fgets(buf, MEM_SIZE, stdin);
        size_t sz = strlen(buf);
        if (sz == MEM_SIZE-1 && buf[sz-1] == '\n') sz--;
        for (int i = 0; i < sz; ++i) {
            if (semop(semid, &wait_write, 1) == -1) {
                perror("semop");
                return 1;
            }
            //printf("R"); /*for demonstrate*/
            //fflush(stdout); /*for demonstrate*/
            shm_ptr[(pw) % (MEM_SIZE)] = buf[i];
            pw = (pw + 1) % MEM_SIZE;
            if (semop(semid, &notify_read, 1) == -1) {
                perror("semop");
                return 1;
            }
            if (buf[i] == '#') ENDFLAG = 0;  // TODO: явно goto написать или оставить так?
        }
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