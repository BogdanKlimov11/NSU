def power(x: float, n: int) -> float:
    result = 1.0
    exponent = abs(n)  # Работаем с положительным показателем

    while exponent > 0:
        if exponent % 2 == 1:  # Если текущий бит 1, умножаем на x
            result *= x
        x *= x  # Умножаем x на себя (x^2, x^4, x^8 и т. д.)
        exponent //= 2  # Сдвигаем степень вправо

    return 1.0 / result if n < 0 else result  # Если n отрицательное, возвращаем 1 / result

# Пример использования
x = float(input("Введите число: "))
n = int(input("Введите степень: "))

print(f"{x}^{n} = {power(x, n)}")
