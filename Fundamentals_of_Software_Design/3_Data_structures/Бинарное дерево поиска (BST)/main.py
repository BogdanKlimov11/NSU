import random

class _Node:
    """Single node of a tree. Keeps references to parent, left, right and some data."""
    def __init__(self, data):
        self.data = data
        self.left = None
        self.right = None
        self.parent = None

class Tree:
    """Tree itself. Keeps root node of a tree and a comparison function."""
    def __init__(self, cmp_func):
        self.root = None
        self.cmp_func = cmp_func
        self._size = 0

    def size(self):
        """Return number of elements."""
        return self._size

    def clear(self):
        """Clear tree but do not destroy tree itself."""
        self.root = None
        self._size = 0

    def find(self, data):
        """Find element with equal data and return its data if any."""
        node = self._find_node(data)
        return node.data if node else None

    def insert(self, data):
        """Insert data into tree and return replaced data if any."""
        if self.root is None:
            self.root = _Node(data)
            self._size += 1
            return None
        
        parent, direction = self._find_insert_position(data)
        if direction == 0:
            replaced = parent.data
            parent.data = data
            return replaced
        
        new_node = _Node(data)
        new_node.parent = parent
        if direction == -1:
            parent.left = new_node
        else:
            parent.right = new_node
        
        self._size += 1
        return None

    def delete(self, data):
        """Delete element with equal data and return its data if any."""
        node = self._find_node(data)
        if node is None:
            return None
        
        deleted_data = node.data
        self._delete_node(node)
        self._size -= 1
        return deleted_data

    def foreach(self, func):
        """Call func for every element's data in tree in infix-order."""
        self._inorder_traversal(self.root, func)
    
    def _find_node(self, data):
        node = self.root
        while node:
            cmp_result = self.cmp_func(data, node.data)
            if cmp_result == 0:
                return node
            elif cmp_result < 0:
                node = node.left
            else:
                node = node.right
        return None
    
    def _find_insert_position(self, data):
        node = self.root
        parent = None
        direction = 0
        
        while node:
            cmp_result = self.cmp_func(data, node.data)
            if cmp_result == 0:
                return node, 0
            parent = node
            if cmp_result < 0:
                node = node.left
                direction = -1
            else:
                node = node.right
                direction = 1
        
        return parent, direction
    
    def _delete_node(self, node):
        if node.left and node.right:
            successor = self._min_value_node(node.right)
            node.data = successor.data
            self._delete_node(successor)
        elif node.left:
            self._replace_node_in_parent(node, node.left)
        elif node.right:
            self._replace_node_in_parent(node, node.right)
        else:
            self._replace_node_in_parent(node, None)
    
    def _replace_node_in_parent(self, node, new_node):
        if node.parent:
            if node == node.parent.left:
                node.parent.left = new_node
            else:
                node.parent.right = new_node
        if new_node:
            new_node.parent = node.parent
        if node == self.root:
            self.root = new_node
    
    def _min_value_node(self, node):
        while node.left:
            node = node.left
        return node
    
    def _inorder_traversal(self, node, func):
        if node:
            self._inorder_traversal(node.left, func)
            func(node.data)
            self._inorder_traversal(node.right, func)

# Testing functions
def cmp_func(a, b):
    return (a > b) - (a < b)

def test_bst():
    tree = Tree(cmp_func)
    assert tree.size() == 0

    assert tree.insert(5) is None
    assert tree.insert(3) is None
    assert tree.insert(7) is None
    assert tree.insert(5) == 5  # Replace existing

    assert tree.size() == 3
    assert tree.find(3) == 3
    assert tree.find(10) is None
    
    assert tree.delete(3) == 3
    assert tree.size() == 2
    assert tree.delete(10) is None
    
    print("BST tests passed!")

test_bst()
