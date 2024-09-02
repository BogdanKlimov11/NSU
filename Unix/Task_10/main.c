#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int execvpe(const char *file, char *const argv[], char *const envp[]) {
    if (clearenv() != 0) {
        perror("clearenv() error");
	return -1;
    }

    for (int i = 0; envp[i]; ++i) {
        if (putenv(envp[i]) != 0) {
	    perror("putenv");
	    return -1;
	}
    }
    return execvp(file, argv);
}

int main(int argc, char* argv[]) {
    char *args[] = {"ARGS1", "ARGS2","ARGS3", NULL};
    char *envp[] = {"ENV1=ddyak", "ENV2=ustal", "ENV3=testit:(", NULL};
    if (execvpe("./printer", args, envp) == -1) {
    	perror("execvpe");
    }
    return -1;
}
