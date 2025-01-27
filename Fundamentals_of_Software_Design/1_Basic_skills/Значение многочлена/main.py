def polynom(x, a, n):
    result = 0.0
    for i in range(n + 1):
        result += a[i] * (x ** i)  # Добавляем a[i] * x^i
    return result


# Пример использования
coefficients = [1, -3, 2]  # Многочлен: 2x^2 - 3x + 1
x_value = 3
degree = 2  # Степень многочлена
print(f"Значение многочлена: {polynom(x_value, coefficients, degree)}")  # 2*3^2 - 3*3 + 1 = 10
