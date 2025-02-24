import random
import time

# Функция для чтения текста из файла и создания массива слов
def load_text(filename):
    with open(filename, 'r', encoding='utf-8') as file:
        text = file.read()
    words = text.split()
    return words

# Функция для генерации случайных предложений
def generate_sentence(words):
    sentence_length = random.randint(3, 8)  # длина предложения от 3 до 8 слов
    sentence = random.sample(words, sentence_length)  # случайные слова
    sentence[0] = sentence[0].capitalize()  # первое слово с заглавной буквы

    # Случайное добавление знака препинания
    punctuation = random.choice([',', '-', ''])
    if punctuation:
        sentence.insert(random.randint(1, sentence_length - 1), punctuation)
    
    # Завершаем предложение точкой или восклицательным знаком
    sentence = ' '.join(sentence) + random.choice(['.', '!'])
    return sentence

# Функция для генерации тренировочного текста
def generate_training_text(words):
    sentences = [generate_sentence(words) for _ in range(random.randint(5, 8))]
    return ' '.join(sentences)

# Функция для проверки ввода пользователя
def check_typing_speed(training_text):
    print("Тренировочный текст:")
    print(training_text)
    print("\nНачните вводить этот текст. Нажмите Enter для начала.")
    input()

    start_time = time.time()  # Засекаем время начала ввода
    typed_text = input("Введите текст: ")
    end_time = time.time()  # Засекаем время окончания ввода

    # Подсчитываем ошибки
    errors = sum(1 for i in range(min(len(training_text), len(typed_text))) if training_text[i] != typed_text[i])
    total_time = end_time - start_time
    typing_speed_chars = len(typed_text) / total_time * 60
    typing_speed_words = len(typed_text.split()) / total_time * 60

    # Выводим статистику
    print("\nСтатистика:")
    print(f"Ошибок: {errors}")
    print(f"Время: {total_time:.2f} секунд")
    print(f"Скорость ввода: {typing_speed_chars:.2f} символов в минуту")
    print(f"Скорость ввода: {typing_speed_words:.2f} слов в минуту")

def main():
    # Загрузка текста из файла
    filename = 'text.txt'  # Укажите путь к своему файлу с текстом
    words = load_text(filename)
    
    # Генерация тренировочного текста
    training_text = generate_training_text(words)
    
    # Проверка навыков печати
    check_typing_speed(training_text)

if __name__ == "__main__":
    main()
