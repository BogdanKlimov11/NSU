#include <sys/msg.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define MAX_SEND_SZ 32

struct msgbuf {
    long mtype;
    char mtext[MAX_SEND_SZ];
};

int main() {
    int mid = msgget(getuid(), IPC_CREAT | 0660);
    if (mid == -1) {
        perror("msgget");
        return 1;
    }
    struct msgbuf buf;
    buf.mtype = 1;
    while (1) {
        scanf("%s", buf.mtext);
        if (msgsnd(mid, &buf, strlen(buf.mtext) + 1, 0) == -1) {
            perror("msgsnd");
            break;
        }
        if (strcmp(buf.mtext, "bye") == 0) break;
    }
    if (msgctl(mid, IPC_RMID, NULL) == -1) {
        perror("msgctl");
        return -1;
    }
    return 0;
}