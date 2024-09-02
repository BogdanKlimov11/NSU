#include <stdio.h>
extern char **environ;

int main(int argc, char *argv[]) {
    printf("ARGV PARAMETERS:\n");
    for (int n = 0; n < argc; ++n)
        printf("\t%d:'%s'\n", n, argv[n]);

    printf("\nENVIRONMENT VARIABLES:\n");
    for (char **p = environ; *p; ++p)
        printf("\t%s\n", *p);
    return 0;
}

