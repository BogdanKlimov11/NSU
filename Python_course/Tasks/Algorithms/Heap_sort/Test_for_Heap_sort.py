from Heap_sort import heap_sort
import random

#Тесты для пиромидальной сортировки

# Случай len = 0:
assert heap_sort([]) == []

# Случай len = 1:
assert heap_sort([3]) == [3]

# Случай len четное:
assert heap_sort([2, 5, 7, 1, 0, 6, 3]) == [0, 1, 2, 3, 5, 6, 7]

# Случай len нечетное:
assert heap_sort([9, 4, 8, 1, 0, 3]) == [0, 1, 3, 4, 8, 9]

# Случай, когда массив уже отсортирован:
assert heap_sort([1, 2, 3, 4, 5, 6]) == [1, 2, 3, 4, 5, 6]

# Случай, когда в массиве одинаковые числа:
assert heap_sort([5, 5, 5, 5, 5]) == [5, 5, 5, 5, 5]

# Случай элементы отрицательные:
assert heap_sort([-4, -9, -1, -7, -3]) == [-9, -7, -4, -3, -1]

# Случай со случайным массивом:
length = random.randrange(26)
arr = []
for i in range(length):
    arr.append(random.randrange(-20, 21))
assert heap_sort(arr) == sorted(arr)
