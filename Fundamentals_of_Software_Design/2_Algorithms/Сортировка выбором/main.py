import random

def selection_sort(arr):
    """Сортировка выбором (изменяет переданный массив)"""
    n = len(arr)
    for i in range(n):
        min_idx = i
        for j in range(i + 1, n):
            if arr[j] < arr[min_idx]:
                min_idx = j
        arr[i], arr[min_idx] = arr[min_idx], arr[i]

# Тестирование
arr1 = []
selection_sort(arr1)
assert arr1 == []

arr2 = [5]
selection_sort(arr2)
assert arr2 == [5]

arr3 = [3, 1, 2]
selection_sort(arr3)
assert arr3 == [1, 2, 3]

arr4 = [5, 4, 3, 2, 1]
selection_sort(arr4)
assert arr4 == [1, 2, 3, 4, 5]

# Тест с генерацией случайных массивов
for _ in range(10):
    test_arr = [random.randint(-100, 100) for _ in range(random.randint(0, 20))]
    sorted_arr = sorted(test_arr)  # Эталонная сортировка
    selection_sort(test_arr)
    assert test_arr == sorted_arr

print("Все тесты пройдены!")
