#include <stdio.h>
#include <stdlib.h>

typedef void *Pointer;

// Функция сравнения для элементов дерева
typedef int (*CmpFunc)(Pointer data1, Pointer data2);

// Узел бинарного дерева поиска
typedef struct tBSTNode {
    Pointer data;
    struct tBSTNode *left;
    struct tBSTNode *right;
} BSTNode;

// Структура самого дерева
typedef struct tBST {
    BSTNode *root;
    CmpFunc cmp_func; // Функция сравнения
} BST;

// Создание пустого дерева
BST *bst_create(CmpFunc cmp_func) {
    BST *tree = (BST *)malloc(sizeof(BST));
    if (!tree) return NULL;
    tree->root = NULL;
    tree->cmp_func = cmp_func;
    return tree;
}

// Очистка узлов дерева (вспомогательная рекурсивная функция)
void bst_clear_nodes(BSTNode *node) {
    if (!node) return;
    bst_clear_nodes(node->left);
    bst_clear_nodes(node->right);
    free(node);
}

// Очистка дерева, но без удаления структуры
void bst_clear(BST *tree) {
    bst_clear_nodes(tree->root);
    tree->root = NULL;
}

// Уничтожение дерева
void bst_destroy(BST *tree) {
    if (!tree) return;
    bst_clear(tree);
    free(tree);
}

// Вставка элемента
BSTNode *bst_insert_node(BSTNode *node, Pointer data, CmpFunc cmp_func) {
    if (!node) {
        BSTNode *new_node = (BSTNode *)malloc(sizeof(BSTNode));
        if (!new_node) return NULL;
        new_node->data = data;
        new_node->left = new_node->right = NULL;
        return new_node;
    }
    if (cmp_func(data, node->data) < 0)
        node->left = bst_insert_node(node->left, data, cmp_func);
    else
        node->right = bst_insert_node(node->right, data, cmp_func);
    return node;
}

// Интерфейсная функция вставки
Pointer bst_insert(BST *tree, Pointer data) {
    if (!tree) return NULL;
    tree->root = bst_insert_node(tree->root, data, tree->cmp_func);
    return data;
}

// Поиск элемента
Pointer bst_find(BST *tree, Pointer data) {
    BSTNode *node = tree->root;
    while (node) {
        int cmp = tree->cmp_func(data, node->data);
        if (cmp == 0) return node->data;
        node = (cmp < 0) ? node->left : node->right;
    }
    return NULL;
}

// Обход дерева (инфиксный порядок)
void bst_foreach_node(BSTNode *node, void (*foreach_func)(Pointer, Pointer), Pointer extra_data) {
    if (!node) return;
    bst_foreach_node(node->left, foreach_func, extra_data);
    foreach_func(node->data, extra_data);
    bst_foreach_node(node->right, foreach_func, extra_data);
}

// Интерфейсная функция обхода
void bst_foreach(BST *tree, void (*foreach_func)(Pointer, Pointer), Pointer extra_data) {
    if (!tree || !foreach_func) return;
    bst_foreach_node(tree->root, foreach_func, extra_data);
}

// Пример функции сравнения для целых чисел
int int_cmp(Pointer a, Pointer b) {
    return (*(int *)a - *(int *)b);
}

// Тестирование дерева
void print_node(Pointer data, Pointer extra) {
    printf("%d ", *(int *)data);
}

int main() {
    BST *tree = bst_create(int_cmp);
    int values[] = {5, 3, 8, 1, 4, 7, 10};
    for (int i = 0; i < 7; i++) bst_insert(tree, &values[i]);
    
    printf("BST inorder traversal: ");
    bst_foreach(tree, print_node, NULL);
    printf("\n");
    
    int key = 4;
    int *found = (int *)bst_find(tree, &key);
    printf("Find %d: %s\n", key, found ? "Found" : "Not Found");
    
    bst_destroy(tree);
    return 0;
}
