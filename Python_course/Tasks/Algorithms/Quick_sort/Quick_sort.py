import random

# Быстрая сортировка
def quick_sort(arr):
    # Если длина массива 1 или 0, то он уже и так отсортирован
    if len(arr) <= 1:
        return arr
    else:
        # Выбираем элемент относительно которого будем распределять элементы
        base = random.choice(arr)
        # Массив элементов меньше base
        left_arr = []
        # Массив элементов больших base
        middle_arr = []
        # Массив элементов равных base
        right_arr = []
        # Распределяем элементы в массивы
        for elem in arr:
            if elem < base:
                left_arr.append(elem)
            elif elem > base:
                right_arr.append(elem)
            else:
                middle_arr.append(elem)
        # Повтаряем процесс для левого и правого массивов
        return quick_sort(left_arr) + middle_arr + quick_sort(right_arr)