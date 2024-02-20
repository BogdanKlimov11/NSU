# Двоичный поиск
def binary_search(arr, key):
    low = 0
    high = len(arr) - 1
    while low <= high:
        mid = (low + high) // 2
        guess = arr[mid]
        if guess == key:
            return mid
        elif guess < key:
            low = mid + 1
        else:
            high = mid - 1
    return None