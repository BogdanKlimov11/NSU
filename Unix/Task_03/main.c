#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>

void print_user_info(){
    printf("Real user-id: %d\n", getuid());
    printf("Effective user-id: %d\n", geteuid());
}

void open_file(const char* s){
    FILE* f;
    if((f = fopen(s, "r"))==NULL){
        perror("Cannot open file\n");
    }
    else{
        fclose(f);
    }
}

int main(int argc, char** argv){
	if(argc != 2){
		printf("Usage: ./main testfile.txt\n");
		return 1;
	}
    print_user_info();
    open_file(argv[1]);
    
    setuid(getuid()); // ?
    print_user_info();
    open_file(argv[1]);

}

