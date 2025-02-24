import math

def find_min(nums):
    return min(nums)

def find_max(nums):
    return max(nums)

def find_mean(nums):
    return sum(nums) / len(nums)

def find_median(nums):
    nums_sorted = sorted(nums)
    n = len(nums_sorted)
    if n % 2 == 1:
        return nums_sorted[n // 2]
    else:
        return (nums_sorted[n // 2 - 1] + nums_sorted[n // 2]) / 2

def find_std_deviation(nums, mean):
    variance = sum((x - mean) ** 2 for x in nums) / len(nums)
    return math.sqrt(variance)

def find_max_consecutive(nums):
    max_count = 1
    count = 1
    for i in range(1, len(nums)):
        if nums[i] == nums[i-1]:
            count += 1
            max_count = max(max_count, count)
        else:
            count = 1
    return max_count

def find_max_monotonic(nums):
    max_length = 1
    current_length = 1

    # Для поиска максимальной длины неубывающего участка
    for i in range(1, len(nums)):
        if nums[i] >= nums[i - 1]:
            current_length += 1
        else:
            max_length = max(max_length, current_length)
            current_length = 1

    max_length = max(max_length, current_length)

    # Для поиска максимальной длины невозрастающего участка
    current_length = 1
    for i in range(1, len(nums)):
        if nums[i] <= nums[i - 1]:
            current_length += 1
        else:
            max_length = max(max_length, current_length)
            current_length = 1

    max_length = max(max_length, current_length)

    return max_length

# Основная программа
def main():
    # Ввод чисел с плавающей точкой
    nums = list(map(float, input("Введите числа с плавающей точкой (через пробел): ").split()))

    if not nums:
        print("Массив пуст!")
        return

    # Подсчет характеристик
    min_val = find_min(nums)
    max_val = find_max(nums)
    mean_val = find_mean(nums)
    median_val = find_median(nums)
    std_dev_val = find_std_deviation(nums, mean_val)
    max_consec_val = find_max_consecutive(nums)
    max_monotonic_val = find_max_monotonic(nums)

    # Вывод результатов
    print(f"Минимальное число: {min_val}")
    print(f"Максимальное число: {max_val}")
    print(f"Среднее арифметическое: {mean_val}")
    print(f"Медиана: {median_val}")
    print(f"Среднеквадратичное отклонение: {std_dev_val}")
    print(f"Максимальное количество подряд одинаковых элементов: {max_consec_val}")
    print(f"Максимальная длина монотонного участка: {max_monotonic_val}")

if __name__ == "__main__":
    main()
