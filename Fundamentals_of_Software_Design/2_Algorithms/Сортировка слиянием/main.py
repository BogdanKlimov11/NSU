from typing import List
import random

def merge_sort(arr: List[int]) -> List[int]:
    """Сортировка слиянием с использованием единственного вспомогательного массива."""
    if len(arr) <= 1:
        return arr

    temp = arr[:]  # Выделяем один вспомогательный массив
    _merge_sort(arr, temp, 0, len(arr))
    return arr

def _merge_sort(arr: List[int], temp: List[int], left: int, right: int):
    """Рекурсивная сортировка слиянием."""
    if right - left <= 1:
        return
    mid = (left + right) // 2
    _merge_sort(arr, temp, left, mid)
    _merge_sort(arr, temp, mid, right)
    merge(arr, temp, left, mid, right)

def merge(arr: List[int], temp: List[int], left: int, mid: int, right: int):
    """Слияние двух отсортированных частей."""
    i, j, k = left, mid, left
    while i < mid and j < right:
        if arr[i] < arr[j]:
            temp[k] = arr[i]
            i += 1
        else:
            temp[k] = arr[j]
            j += 1
        k += 1
    while i < mid:
        temp[k] = arr[i]
        i += 1
        k += 1
    while j < right:
        temp[k] = arr[j]
        j += 1
        k += 1
    arr[left:right] = temp[left:right]

# Тесты
def test():
    assert merge_sort([]) == []
    assert merge_sort([42]) == [42]
    assert merge_sort([5, 2, 8, 3, 1]) == [1, 2, 3, 5, 8]
    assert merge_sort([3, 3, 3, 3]) == [3, 3, 3, 3]
    assert merge_sort([-2, -5, 0, 7, 1]) == [-5, -2, 0, 1, 7]
    print("Все тесты пройдены!")

# Генерация случайных массивов и тестирование
def random_test(num_tests: int, max_size: int, max_value: int):
    for _ in range(num_tests):
        size = random.randint(0, max_size)
        arr = [random.randint(-max_value, max_value) for _ in range(size)]
        assert merge_sort(arr[:]) == sorted(arr)
    print("Все случайные тесты пройдены!")

if __name__ == "__main__":
    test()
    random_test(100, 1000, 1000)
