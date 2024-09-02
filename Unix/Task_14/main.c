#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <memory.h>

#define MAXFDC 8
#define BUFSZ 256
#define TIME_OUT 1

int main(int argc, char *argv[]) {
    size_t fdc = 0;
    FILE *fd[MAXFDC];

    for (size_t i = 1; i < argc; ++i) {
        if ((fd[fdc] = fopen(argv[i], "rb")) != NULL) {
            if (++fdc == MAXFDC) break;
        } else {
            perror(argv[i]);
        }
    }

    fd_set rfds;
    char buf[BUFSZ];
    struct timeval timeout;

    while (fdc) {
        for (size_t i = 0; i < fdc; ++i) {
            timeout.tv_sec = TIME_OUT;
            timeout.tv_usec = 0;
            FD_ZERO(&rfds);
            FD_SET(fileno(fd[i]), &rfds);
            if (select(fileno(fd[i]) + 1, &rfds, NULL, NULL, &timeout) > 0) {
                if (fgets(buf, BUFSZ, fd[i]) != 0) {
                    write(1, buf, strlen(buf));
                } else {
                    /* Закрывая файл уменьшим число открытых файловых дескрипторов fdc
                     * один дескриптор останется вне диапазона обхода цикла, поменяем его
                     * с только что закрытым дескриптором i */
                    close(fileno(fd[i]));
                    fd[i--] = fd[--fdc];
                }
            }
        }
    }
    return 0;
}
