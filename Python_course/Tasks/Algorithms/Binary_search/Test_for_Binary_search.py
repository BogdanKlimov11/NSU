from Binary_search import binary_search

# Тесты для двоичного поиска

# Случай size = 0:
assert binary_search([], 4) == None

# Случай size = 1, элемента нет в массиве:
assert binary_search([1], 4) == None

# Случай size = 1, элемент есть в массиве:
assert binary_search([4], 4) == 0

# Случай size четное:
assert binary_search([0, 2, 3, 5, 7, 8, 9], 5) == 3

# Случай size нечетное:
assert binary_search([1, 1, 3, 4, 7, 8, 9], 7) == 4

# Случай, когда элемента нет в массиве:
assert binary_search([1, 1, 3, 4, 7, 8, 9], 2) == None
