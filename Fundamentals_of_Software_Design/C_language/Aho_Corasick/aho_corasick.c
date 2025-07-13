#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "aho_corasick.h"

#define ALPHABET_SIZE 256

typedef struct TrieNode {
    struct TrieNode* children[ALPHABET_SIZE];
    struct TrieNode* fail;
    int is_end;
    size_t pattern_index;
} TrieNode;

static TrieNode* create_node() {
    TrieNode* node = (TrieNode*)calloc(1, sizeof(TrieNode));
    if (!node) return NULL;
    node->fail = NULL;
    node->is_end = 0;
    node->pattern_index = 0;
    return node;
}

static void insert_pattern(TrieNode* root, const char* pattern, size_t index) {
    TrieNode* node = root;
    for (size_t i = 0; pattern[i]; i++) {
        int ch = (unsigned char)pattern[i];
        if (!node->children[ch]) {
            node->children[ch] = create_node();
        }
        node = node->children[ch];
    }
    node->is_end = 1;
    node->pattern_index = index;
}

static void build_fail_links(TrieNode* root) {
    TrieNode* queue[ALPHABET_SIZE];
    int front = 0, rear = 0;
    
    root->fail = root;
    for (int i = 0; i < ALPHABET_SIZE; i++) {
        if (root->children[i]) {
            root->children[i]->fail = root;
            queue[rear++] = root->children[i];
        }
    }
    
    while (front < rear) {
        TrieNode* current = queue[front++];
        
        for (int i = 0; i < ALPHABET_SIZE; i++) {
            if (current->children[i]) {
                TrieNode* fail = current->fail;
                while (fail != root && !fail->children[i]) {
                    fail = fail->fail;
                }
                
                if (fail->children[i]) {
                    current->children[i]->fail = fail->children[i];
                } else {
                    current->children[i]->fail = root;
                }
                
                queue[rear++] = current->children[i];
            }
        }
    }
}

SearchResult substring_search(const char* text, const char** patterns, size_t pattern_count) {
    SearchResult result = {NULL, 0};
    if (!text || !patterns || pattern_count == 0) return result;
    
    TrieNode* root = create_node();
    if (!root) return result;
    
    for (size_t i = 0; i < pattern_count; i++) {
        if (patterns[i]) {
            insert_pattern(root, patterns[i], i);
        }
    }
    
    build_fail_links(root);
    
    size_t capacity = 16;
    result.positions = (int*)malloc(capacity * sizeof(int));
    if (!result.positions) {
        // TODO: Free trie nodes
        return result;
    }
    
    TrieNode* current = root;
    for (size_t i = 0; text[i]; i++) {
        int ch = (unsigned char)text[i];
        
        while (current != root && !current->children[ch]) {
            current = current->fail;
        }
        
        if (current->children[ch]) {
            current = current->children[ch];
        }
        
        TrieNode* temp = current;
        while (temp != root) {
            if (temp->is_end) {
                if (result.count >= capacity) {
                    capacity *= 2;
                    int* new_positions = (int*)realloc(result.positions, capacity * sizeof(int));
                    if (!new_positions) {
                        free(result.positions);
                        // TODO: Free trie nodes
                        result.positions = NULL;
                        result.count = 0;
                        return result;
                    }
                    result.positions = new_positions;
                }
                result.positions[result.count++] = i - strlen(patterns[temp->pattern_index]) + 1;
            }
            temp = temp->fail;
        }
    }
    
    // TODO: Free trie nodes
    return result;
}
