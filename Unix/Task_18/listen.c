#include <sys/msg.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define MAX_RCV_SZ 32

struct msgbuf {
    long mtype;
    char mtext[MAX_RCV_SZ];
};

int main() {
    int queue = msgget(getuid(), 0);
    if (queue == -1) {
        perror("msgget");
        return 1;
    }
    struct msgbuf buf;
    ssize_t res;
    while (1) {
        if ((res = msgrcv(queue, &buf, MAX_RCV_SZ, 1, 0)) == -1) break;
        printf("%s: type %li, length %zi\n", buf.mtext, buf.mtype, res);
        if (!strcmp(buf.mtext, "bye")) break;
    }
    return 0;
}