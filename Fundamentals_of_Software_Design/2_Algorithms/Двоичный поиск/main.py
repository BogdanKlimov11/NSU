def binary_search(arr, key):
    """Бинарный поиск в отсортированном массиве"""
    left, right = 0, len(arr) - 1
    while left <= right:
        mid = (left + right) // 2
        if arr[mid] == key:
            return mid
        elif arr[mid] < key:
            left = mid + 1
        else:
            right = mid - 1
    return -1  # Элемент не найден

# Тесты
arr1 = []
assert binary_search(arr1, 10) == -1  # Пустой массив

arr2 = [5]
assert binary_search(arr2, 5) == 0  # Один элемент (есть)
assert binary_search(arr2, 10) == -1  # Один элемент (нет)

arr3 = [1, 3, 5, 7, 9]
assert binary_search(arr3, 1) == 0  # Первый элемент
assert binary_search(arr3, 9) == 4  # Последний элемент
assert binary_search(arr3, 5) == 2  # Средний элемент
assert binary_search(arr3, 6) == -1  # Элемента нет

arr4 = [2, 4, 6, 8, 10, 12]
assert binary_search(arr4, 6) == 2  # Четный размер
assert binary_search(arr4, 12) == 5  # Последний элемент
assert binary_search(arr4, 3) == -1  # Элемента нет

print("Все тесты пройдены!")
