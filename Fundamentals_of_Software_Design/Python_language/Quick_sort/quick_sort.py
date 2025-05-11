import random

def quick_sort(arr):
    if len(arr) <= 1:
        return arr
    
    def partition(low, high, arr):
        pivot_idx = random.randint(low, high)
        arr[pivot_idx], arr[high] = arr[high], arr[pivot_idx]
        
        pivot = arr[high]
        i = low - 1
        
        for j in range(low, high):
            if arr[j] <= pivot:
                i += 1
                arr[i], arr[j] = arr[j], arr[i]
        
        arr[i + 1], arr[high] = arr[high], arr[i + 1]
        return i + 1
    
    def quick_sort_helper(low, high, arr):
        if low < high:
            pi = partition(low, high, arr)
            quick_sort_helper(low, pi - 1, arr)
            quick_sort_helper(pi + 1, high, arr)
    
    arr_copy = arr.copy()
    quick_sort_helper(0, len(arr_copy) - 1, arr_copy)
    return arr_copy
