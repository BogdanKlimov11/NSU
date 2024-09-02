#include "vector.h"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/msg.h>
#include <memory.h>

#define MAX_SEND_SZ 32
#define REG_TYPE 1
#define END_TYPE 2

struct msgbuf {
    long mtype;
    pid_t pid;
    char mtext[MAX_SEND_SZ];
};

int updListeners(int queue, struct vector *pids) {
    struct msgbuf buf;
    while (msgrcv(queue, &buf, sizeof(pid_t), REG_TYPE, IPC_NOWAIT) != -1) {
        if (addItem(pids, buf.pid) != 0) {
            return 1;
        }
    }
    return 0;
}

int sendMessages(int queue, struct vector *pids) {
    struct msgbuf buf;
    while (1) {
        size_t length;
        if (fgets(buf.mtext, MAX_SEND_SZ, stdin) == NULL) {
            length = 0;
        } else {
            length = strlen(buf.mtext);
            if (buf.mtext[length - 1] == '\n') {
                buf.mtext[length - 1] = '\0';
            } else {
                ++length;
            }
        }
        if (updListeners(queue, pids) != 0) {
            return 1;
        }
        for (int i = 0; i < pids->firstFree; ++i) {
            (&buf)->mtype = pids->vec[i];
            if (msgsnd(queue, &buf, length + sizeof(pid_t), 0) == -1) {
                perror("msgsnd");
                return 1;
            }
        }
        if (!length) break;
    }
    return 0;
}

int waitAnswers(int queue, struct vector *pids) {
    struct msgbuf buf;
    while (pids->firstFree > 0) {
        if (msgrcv(queue, &buf, sizeof(pid_t), END_TYPE, 0) == -1) {
            perror("msgrcv");
            return 1;
        }
        int index = findItem(pids, buf.pid);
        if (removeItem(pids, index) != 0) {
            return 1;
        }
    }
    return 0;
}

int closeQueue(int queue) {
    if (msgctl(queue, IPC_RMID, NULL) == -1) {
        perror("msgctl");
        return 1;
    }
    return 0;
}

int main() {
    key_t queueKey = ftok("send.c", 22);
    if (queueKey == -1) {
        perror("ftok");
        return 1;
    }
    int queue = msgget(queueKey, IPC_CREAT | 0660);
    if (queue == -1) {
        perror("msgget");
        return 1;
    }
    struct vector *pids = initVector();
    if (sendMessages(queue, pids) != 0) {
        closeQueue(queue);  // Как обрабатывать ошибки ошибок?
        destructorVector(pids);
        return 1;
    }
    if (waitAnswers(queue, pids) != 0) {
        closeQueue(queue);
        destructorVector(pids);
        return 1;
    }
    if (closeQueue(queue) != 0) {
        destructorVector(pids);
        return 1;
    }
    destructorVector(pids);
    return 0;
}
