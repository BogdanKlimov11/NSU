import random

def bubble_sort(arr):
    """Сортировка пузырьком (изменяет переданный массив)"""
    n = len(arr)
    for i in range(n - 1):
        swapped = False
        for j in range(n - 1 - i):
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
                swapped = True
        if not swapped:  # Если не было перестановок, массив уже отсортирован
            break

# Тестирование
arr1 = []
bubble_sort(arr1)
assert arr1 == []

arr2 = [5]
bubble_sort(arr2)
assert arr2 == [5]

arr3 = [3, 1, 2]
bubble_sort(arr3)
assert arr3 == [1, 2, 3]

arr4 = [5, 4, 3, 2, 1]
bubble_sort(arr4)
assert arr4 == [1, 2, 3, 4, 5]

# Тест с генерацией случайных массивов
for _ in range(10):
    test_arr = [random.randint(-100, 100) for _ in range(random.randint(0, 20))]
    sorted_arr = sorted(test_arr)  # Эталонная сортировка
    bubble_sort(test_arr)
    assert test_arr == sorted_arr

print("Все тесты пройдены!")
