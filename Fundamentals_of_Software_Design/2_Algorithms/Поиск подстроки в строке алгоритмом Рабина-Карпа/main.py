def my_strstr(haystack, needle):
    # Размеры строк
    n = len(haystack)
    m = len(needle)
    
    if m == 0:
        return 0  # Пустая подстрока всегда на первой позиции
    if n < m:
        return -1  # Если строка меньше подстроки, то совпадения нет
    
    # Параметры хеширования
    BASE = 256  # Основание для хеширования (число символов в алфавите, например, ASCII)
    MOD = 101   # Модуль для хеширования (простое число для уменьшения коллизий)
    
    # Вычисление хеша для строки
    def compute_hash(s, length):
        hash_val = 0
        for i in range(length):
            hash_val = (hash_val * BASE + ord(s[i])) % MOD
        return hash_val
    
    # Вычисление хеша для needle и первого окна в haystack
    needle_hash = compute_hash(needle, m)
    haystack_hash = compute_hash(haystack, m)
    
    # Предсчитываем степень BASE^(m-1) для использования в обновлении хеша
    base_m = 1
    for i in range(m - 1):
        base_m = (base_m * BASE) % MOD
    
    # Поиск подстроки
    for i in range(n - m + 1):
        # Если хеши совпали, проверяем символы
        if haystack_hash == needle_hash:
            if haystack[i:i + m] == needle:
                return i  # Совпадение найдено, возвращаем индекс
        # Если не на последней позиции, сдвигаем окно
        if i < n - m:
            haystack_hash = (haystack_hash - ord(haystack[i]) * base_m) % MOD
            haystack_hash = (haystack_hash * BASE + ord(haystack[i + m])) % MOD
    
    return -1  # Подстрока не найдена

# Пример тестирования
haystack = "this is a simple example"
needle = "simple"

result = my_strstr(haystack, needle)
if result != -1:
    print(f"Подстрока найдена на позиции: {result}")
else:
    print("Подстрока не найдена")
