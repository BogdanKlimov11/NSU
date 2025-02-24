import random
import datetime

# Функция для преобразования числа в текст
def number_to_text(num):
    numbers = {
        0: 'ноль', 1: 'один', 2: 'два', 3: 'три', 4: 'четыре', 5: 'пять', 6: 'шесть', 7: 'семь', 8: 'восемь', 9: 'девять',
        10: 'десять', 11: 'одиннадцать', 12: 'двенадцать', 13: 'тринадцать', 14: 'четырнадцать', 15: 'пятнадцать',
        16: 'шестнадцать', 17: 'семнадцать', 18: 'восемнадцать', 19: 'девятнадцать', 20: 'двадцать', 30: 'тридцать',
        40: 'сорок', 50: 'пятьдесят', 60: 'шестидесяти', 70: 'семидесяти', 80: 'восьмидесяти', 90: 'девяносто'
    }
    
    if num <= 20:
        return numbers[num]
    elif num < 30:
        return f'двадцать {numbers[num - 20]}'
    elif num < 40:
        return f'тридцать {numbers[num - 30]}'
    elif num < 50:
        return f'сорок {numbers[num - 40]}'
    elif num < 60:
        return f'пятьдесят {numbers[num - 50]}'
    else:
        return f'шестидесяти {numbers[num - 60]}'

# Функция для получения текстового представления времени
def time_to_text(hour, minute):
    hour_text = number_to_text(hour)
    if minute == 0:
        return f'{hour_text} часов ровно'
    
    minute_text = number_to_text(minute)
    
    if minute <= 30:
        return f'{hour_text} часов {minute_text} минут'
    elif minute > 30:
        # Преобразуем в вариант без пятнадцати
        return f'{number_to_text((hour + 1) % 12)} часов без пятнадцати минут {60 - minute}'

# Функция для получения дня недели и даты
def date_to_text():
    days_of_week = ['понедельник', 'вторник', 'среда', 'четверг', 'пятница', 'суббота', 'воскресенье']
    today = datetime.datetime.today()
    return f'{days_of_week[today.weekday()]}, {today.strftime("%d %B %Y")}'

# Основная функция
def speaking_clock():
    # Получаем текущее время
    now = datetime.datetime.now()
    hour = now.hour
    minute = now.minute
    time_in_words = time_to_text(hour, minute)
    date_in_words = date_to_text()

    # Выводим время и дату
    print(f"Сегодня: {date_in_words}")
    print(f"Текущее время: {time_in_words}")

if __name__ == '__main__':
    speaking_clock()
