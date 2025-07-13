class TrieNode:
    __slots__ = ['children', 'fail', 'output']

    def __init__(self):
        self.children = {}
        self.fail = None
        self.output = []

def build_trie(patterns):
    root = TrieNode()
    for pattern in patterns:
        node = root
        for char in pattern:
            if char not in node.children:
                node.children[char] = TrieNode()
            node = node.children[char]
        node.output.append(pattern)
    return root

def build_fail_links(root):
    from collections import deque
    queue = deque()
    
    root.fail = root
    for node in root.children.values():
        node.fail = root
        queue.append(node)
    
    while queue:
        current = queue.popleft()
        
        for char, node in current.children.items():
            fail = current.fail
            while char not in fail.children and fail != root:
                fail = fail.fail
            
            if char in fail.children:
                node.fail = fail.children[char]
            else:
                node.fail = root
            
            node.output += node.fail.output
            queue.append(node)
    
    return root

def aho_corasick_search(text, patterns):
    if not patterns or not text:
        return []
    
    root = build_trie(patterns)
    root = build_fail_links(root)
    
    result = []
    current = root
    
    for i, char in enumerate(text):
        while char not in current.children and current != root:
            current = current.fail
        
        if char in current.children:
            current = current.children[char]
        
        for pattern in current.output:
            result.append((i - len(pattern) + 1, pattern))
    
    return [pos for pos, _ in sorted(result)]

def substring_search(text, pattern):
    if not pattern:
        return list(range(len(text) + 1))
    if not text:
        return []
    
    return [pos for pos in aho_corasick_search(text, [pattern])]
