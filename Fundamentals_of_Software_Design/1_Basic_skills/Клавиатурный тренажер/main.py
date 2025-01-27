import random
import string

def generate_random_string(length=10):
    """Генерирует случайную строку из печатных символов."""
    return ''.join(random.choices(string.ascii_letters + string.digits + string.punctuation, k=length))

def typing_trainer():
    """Основная функция клавиатурного тренажера."""
    random_string = generate_random_string()
    print(f"Введите следующую строку: {random_string}")

    user_input = ""
    index = 0

    while index < len(random_string):
        char = input(f"Введите символ '{random_string[index]}': ")
        if char == random_string[index]:
            user_input += char
            index += 1
        else:
            print("Неправильный символ. Попробуйте снова.")

    print(f"Поздравляем! Вы правильно ввели строку: {user_input}")

# Запуск программы
typing_trainer()
