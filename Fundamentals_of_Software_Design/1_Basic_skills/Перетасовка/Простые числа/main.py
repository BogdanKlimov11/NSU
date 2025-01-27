def sieve_of_eratosthenes(N):
    # Массив для отметки простых чисел
    is_prime = [True] * (N + 1)
    
    # 0 и 1 не являются простыми числами
    is_prime[0] = is_prime[1] = False
    
    # Алгоритм решета Эратосфена
    for i in range(2, int(N**0.5) + 1):
        if is_prime[i]:
            for j in range(i * i, N + 1, i):
                is_prime[j] = False
    
    # Выводим простые числа
    primes = [i for i in range(2, N + 1) if is_prime[i]]
    return primes

# Ввод числа N
N = int(input("Введите число N: "))

# Получаем простые числа и выводим их
primes = sieve_of_eratosthenes(N)
print(f"Простые числа, меньшие или равные {N}:")
print(" ".join(map(str, primes)))
