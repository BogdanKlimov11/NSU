def check_parentheses(expression):
    """Функция возвращает True, если выражение правильное,
       или False в противном случае."""
    balance = 0
    for char in expression:
        if char == '(':
            balance += 1
        elif char == ')':
            balance -= 1
        if balance < 0:  # Если закрывающих скобок больше, чем открывающих
            return False
    return balance == 0  # Баланс должен быть нулевым

# Тесты
assert check_parentheses("") == True
assert check_parentheses("()") == True
assert check_parentheses("((()))()(())") == True
assert check_parentheses("(()())") == True
assert check_parentheses(")(") == False
assert check_parentheses("((())))") == False
assert check_parentheses("(()") == False

print("Все тесты пройдены.")
