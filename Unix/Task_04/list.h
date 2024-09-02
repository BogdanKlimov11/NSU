#pragma once

typedef struct Node{
    char* data;
    struct Node* next;
} Node;

Node* add_node(Node* node, char* text);
void print_nodes(Node* head);
void delete_nodes(Node* head);


