#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "list.h"

int main(int argc, char* argv[]){
    char buffer[BUFSIZ];
    Node* node = NULL;
    while(fgets(buffer, BUFSIZ, stdin) != NULL && buffer[0] != '.'){
        node = add_node(node, (char*)buffer);
	if(node == NULL){
		break;
	}
    }
    print_nodes(node);
    delete_nodes(node);
    return 0;
}
