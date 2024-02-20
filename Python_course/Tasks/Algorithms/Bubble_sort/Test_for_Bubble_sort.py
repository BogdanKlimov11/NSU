from Bubble_sort import bubble_sort
import random

#Тесты для пузырьковой сортировки

# Случай len = 0:
assert bubble_sort([]) == []

# Случай len = 1:
assert bubble_sort([3]) == [3]

# Случай len четное:
assert bubble_sort([2, 5, 7, 1, 0, 6, 3]) == [0, 1, 2, 3, 5, 6, 7]

# Случай len нечетное:
assert bubble_sort([9, 4, 8, 1, 0, 3]) == [0, 1, 3, 4, 8, 9]

# Случай, когда массив уже отсортирован:
assert bubble_sort([1, 2, 3, 4, 5, 6]) == [1, 2, 3, 4, 5, 6]

# Случай, когда в массиве одинаковые числа:
assert bubble_sort([5, 5, 5, 5, 5]) == [5, 5, 5, 5, 5]

# Случай элементы отрицательные:
assert bubble_sort([-4, -9, -1, -7, -3]) == [-9, -7, -4, -3, -1]

# Случай со случайным массивом:
length = random.randrange(26)
arr = []
for i in range(length):
    arr.append(random.randrange(-20, 21))
assert bubble_sort(arr) == sorted(arr)
