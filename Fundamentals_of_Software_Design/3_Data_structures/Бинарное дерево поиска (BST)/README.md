# Задача «Бинарное дерево поиска (BST)»

Реализовать модуль для работы с бинарными деревьями поиска (binary
search tree). Подразумеваются обычные деревья, без балансировки.

### Python interface:

```python
class _Node:
    """Single node of a tree. Keeps references to parent, left, right and some data."""
    pass

class Tree:
    """Tree itself. Keeps root node of a tree and a comparision function."""
    pass

def create(cmp_func):
    """Create empty tree."""
    ...

def clear(tree):
    """Clear tree but do not destroy tree itself."""
    ...

def size(tree):
    """Return number of elements."""
    ...

def find(tree, data):
    """Find element with equal data and return its data if any."""
    ...

def insert(tree, data):
    """Insert data into tree and return replaced data if any."""
    ...

def delete(tree, data):
    """Delete element with equal data and return its data if any."""
    ...

def foreach(tree, func):
    """Call func for every element's data in tree in infix-order."""
    ...
```

### C interface:

```c
typedef void * Pointer;

typedef int (*CmpFunc)(Pointer data1, Pointer data2);

typedef struct tBSTNode {
    Pointer data;
    /* ... */
} BSTNode;

typedef struct tBST {
    BSTNode *root;
    CmpFunc cmp_func; /* all data comparisons should be done
                         with help of this func! */
    /* ... */
} BST;

// Create empty tree
BST * bst_create(CmpFunc cmp_func);

// Clear tree but do not destroy tree struct
void bst_clear(BST *tree);

// Completely destroy tree
void bst_destroy(BST *tree);

size_t bst_size(BST *tree);

// Find element with equal data and return its data if any else NULL
Pointer bst_find(BST *tree, Pointer data);

// Return data which was replaced by this insertion if any else NULL
Pointer bst_insert(BST *tree, Pointer data);

// Delete node with equal data and return its data if any else NULL
Pointer bst_delete(BST *tree, Pointer data);

// Call foreach_func for every node's data in tree passing given extra_data
void bst_foreach(BST *tree,
                 void (*foreach_func)(Pointer data, Pointer extra_data),
                 Pointer extra_data);
```

### Применение

Для лучшего понимания нужно реализовать абстрактный тип данных
ассоциативный массив (отображение, словарь, map) с использованием
данного дерева.

Python interface: в таком случае в дереве могут храниться пары из ключа
и значения, причем функция сравнения работает с первым, а при запросе
данные выдается второе.

C interface: В таком случае в дереве будут храниться указатели на
`Entry`, содержащие ключ, по которому работает функция сравнения, и
некоторое значение, соответствующее ключу.

```c
typedef struct tEntry {
    Pointer key;
    Pointer value;
}
```

### Примечание

Каждая функция модуля должна быть протестирована с помощью `assert`.

Также предусмотреть нагрузочное тестирование: добавление большого числа
случайных элементов в дерево, затем извлечение всех элементов, которые
были добавлены.
