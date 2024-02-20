# Функция сртоит пирамиду из arr
def __build_heap(arr, n, i):
    root = i
    left = 2 * i + 1
    right = 2 * i + 2
    if left < n and arr[i] < arr[left]:
        root = left
    if right < n and arr[root] < arr[right]:
        root = right
    if root != i:
        arr[i], arr[root] = arr[root], arr[i]
        __build_heap(arr, n, root)

# Пирамидальная сортировка
def heap_sort(arr):
    n = len(arr)
    for i in range(n // 2 - 1, -1, -1):
        __build_heap(arr, n, i)
    for i in range(n - 1, 0, -1):
        arr[i], arr[0] = arr[0], arr[i]
        __build_heap(arr, i, 0)
    return arr