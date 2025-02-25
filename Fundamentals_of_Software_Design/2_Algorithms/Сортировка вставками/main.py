import random

def insertion_sort(arr):
    """Сортировка вставками (меняет переданный массив)"""
    for i in range(1, len(arr)):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key

# Тестирование
arr1 = []
insertion_sort(arr1)
assert arr1 == []

arr2 = [5]
insertion_sort(arr2)
assert arr2 == [5]

arr3 = [3, 1, 2]
insertion_sort(arr3)
assert arr3 == [1, 2, 3]

arr4 = [5, 4, 3, 2, 1]
insertion_sort(arr4)
assert arr4 == [1, 2, 3, 4, 5]

# Тест с генерацией случайных массивов
for _ in range(10):
    test_arr = [random.randint(-100, 100) for _ in range(random.randint(0, 20))]
    sorted_arr = sorted(test_arr)  # Эталонная сортировка
    insertion_sort(test_arr)
    assert test_arr == sorted_arr

print("Все тесты пройдены!")
