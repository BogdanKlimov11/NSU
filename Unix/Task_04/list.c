#include "list.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static Node* create_node(char* text){
    Node* node = malloc(sizeof(Node));

    if(node == NULL){
        perror("Can not allocate memory");
        return NULL;
    }

    node->next = NULL;
    node->data = malloc((strlen(text) + 1) * sizeof(char ));

    if (node->data == NULL) {
        perror("Can not allocate memory for string");
        return NULL;
    }

    strcpy(node->data, text);
    return node;
}

Node* add_node(Node* node, char* text){
    if(node == NULL){
        return create_node(text);
    }
    else{
        Node* current_node = node;
        while(current_node->next != NULL){
            current_node = current_node->next;
        }
        current_node->next = create_node(text);
	if(current_node->next == NULL){
		return NULL;
	}
        return node;
    }
}

void delete_nodes(Node* head){
    while (head != NULL){
        Node* current_node = head->next;
	free(head->data);
        free(head);
        head = current_node;
    }
}

void print_nodes(Node *head){
    while (head != NULL){
        printf("%s", head->data);
        head = head->next;
    }
}
