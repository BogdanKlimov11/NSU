class PrimeFactor:
    def __init__(self, prime, power):
        self.prime = prime  # Простое число (множитель)
        self.power = power  # Степень множителя
        self.next = None  # Указатель на следующий элемент списка

def prime_decompose(n):
    if n == 0 or n == 1:
        # Особые случаи для 0 и 1
        return PrimeFactor(n, 1)
    
    head = None  # Головной элемент списка
    tail = None  # Хвостовой элемент списка
    
    # Разложение числа на простые множители
    for i in range(2, int(n**0.5) + 1):
        power = 0
        while n % i == 0:
            n //= i
            power += 1
        if power > 0:
            # Добавляем новый элемент в список
            node = PrimeFactor(i, power)
            if tail:
                tail.next = node
                tail = node
            else:
                head = tail = node
    
    # Если осталось простое число больше 1
    if n > 1:
        node = PrimeFactor(n, 1)
        if tail:
            tail.next = node
        else:
            head = node
    
    return head

def prime_release(head):
    # Для Python не требуется явно освобождать память, но можно реализовать обход списка
    current = head
    while current:
        temp = current
        current = current.next
        del temp

def print_prime_factors(head):
    # Печать разложения числа на простые множители
    current = head
    while current:
        print(f"{current.prime}^{current.power}", end=" ")
        current = current.next
    print()

# Пример использования
if __name__ == "__main__":
    n = int(input("Введите число для разложения: "))
    factors = prime_decompose(n)
    print_prime_factors(factors)
    prime_release(factors)
