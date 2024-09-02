#include <stdio.h>
#include <unistd.h>
#include <sys/msg.h>

#define MAX_RCV_SZ 32
#define REG_TYPE 1
#define END_TYPE 2

struct msgbuf {
    long mtype;
    pid_t pid;
    char mtext[MAX_RCV_SZ];
};

int main(int argc, char **argv) {
    key_t queueKey = ftok("send.c", 22);
    if (queueKey == -1) {
        perror("ftok");
        return 1;
    }
    int queue = msgget(queueKey, 0);
    if (queue == -1) {
        perror("msgget");
        return 1;
    }
    pid_t pid = getpid();
    struct msgbuf buf = {REG_TYPE, pid, ""};
    if (msgsnd(queue, &buf, sizeof(pid_t), 0) == -1) {
        perror("msgsnd:reg");
        return 1;
    }
    struct msgbuf msg;
    ssize_t code;
    while ((code = msgrcv(queue, &msg, MAX_RCV_SZ + sizeof(pid_t), pid, 0)) >= 0) {
        if (code == sizeof(pid_t)) break;
        printf("%s: %s\n", argv[0], msg.mtext);
    }
    if (code == -1) {
        perror("msgrcv");
        return 1;
    }
    struct msgbuf answer = {END_TYPE, pid, ""};
    if (msgsnd(queue, &answer, sizeof(pid_t), 0) == -1) {
        perror("msgsnd:end");
        return 1;
    }
    return 0;
}
