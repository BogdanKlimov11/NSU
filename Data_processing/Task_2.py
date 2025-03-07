import numpy as np
import matplotlib.pyplot as plt
import time

# Анализ сигнала косинуса с частотами 50 и 150 Гц
fs = 1000 # Частота дискретизации в герцах (количество отсчётов в секунду)
t = np.linspace(0, 1, fs, endpoint=False) # Временной массив от 0 до 1 с с шагом 1/fs, без включения конечной точки
x = np.cos(2 * np.pi * 50 * t) + np.cos(2 * np.pi * 150 * t) # Сигнал — сумма косинусов с частотами 50 Гц и 150 Гц

# Определение функции медленного дискретного преобразования Фурье (DFT)
def DFT_slow(x):
    """Медленное вычисление дискретного преобразования Фурье (DFT)"""
    N = len(x) # Длина входного сигнала
    n = np.arange(N) # Массив индексов от 0 до N-1 (временные отсчёты)
    k = n.reshape((N, 1)) # Массив частотных индексов как вектор-столбец
    e = np.exp(-2j * np.pi * k * n / N) # Экспоненциальная матрица e^(-j2πkn/N)
    return np.dot(e, x) # Матричное умножение для получения спектра

# Сравнение времени выполнения FFT и DFT_slow и построение спектров
start_time = time.time() # Засекаем время начала выполнения FFT
fft_result = np.fft.fft(x) # Вычисляем спектр с помощью быстрого преобразования Фурье (FFT)
fft_time = time.time() - start_time # Вычисляем время выполнения FFT

start_time = time.time() # Засекаем время начала выполнения DFT_slow
dft_result = DFT_slow(x) # Вычисляем спектр с помощью медленного DFT
dft_time = time.time() - start_time # Вычисляем время выполнения DFT_slow

freq = np.fft.fftfreq(len(t), 1/fs) # Вычисляем частотный массив (Гц) на основе длины сигнала и шага времени
plt.figure(figsize=(12, 4)) # Создаём новую фигуру для графика размером 12x4 дюйма
plt.plot(freq[:len(freq)//2], np.abs(fft_result)[:len(freq)//2], 'b-', label='FFT') # Спектр FFT (синяя линия)
plt.plot(freq[:len(freq)//2], np.abs(dft_result)[:len(freq)//2], 'r--', label='DFT_slow') # Спектр DFT_slow (красный пунктир)
plt.title('Дискретный спектр сигнала (FFT и DFT_slow)') # Заголовок графика
plt.xlabel('Частота (Гц)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.legend() # Отображаем легенду с подписями линий
plt.grid(True) # Добавляем сетку на график
print(f"Время FFT: {fft_time:.6f} сек") # Выводим время выполнения FFT с точностью до 6 знаков
print(f"Время DFT_slow: {dft_time:.6f} сек") # Выводим время выполнения DFT_slow

# Проверка восстановления сигнала с помощью обратного FFT
x_restored = np.fft.ifft(fft_result) # Выполняем обратное FFT для восстановления сигнала
plt.figure(figsize=(12, 4)) # Создаём новую фигуру размером 12x4 дюйма
plt.plot(t, x.real, 'b-', label='Исходный сигнал') # Исходный сигнал (синяя линия)
plt.plot(t, x_restored.real, 'r--', label='Восстановленный сигнал') # Восстановленный сигнал (красный пунктир)
plt.title('Сравнение исходного и восстановленного сигнала') # Заголовок графика
plt.xlabel('Время (с)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.legend() # Отображаем легенду
plt.grid(True) # Добавляем сетку

# Добавление белого шума и анализ его влияния
noise = np.random.normal(0, 1, x.shape) # Генерируем белый гауссовский шум (ср. значение 0, дисперсия 1)
x_noisy = x + noise # Добавляем шум к исходному сигналу
fft_noisy = np.fft.fft(x_noisy) # Вычисляем спектр зашумленного сигнала
x_noisy_restored = np.fft.ifft(fft_noisy) # Восстанавливаем зашумленный сигнал через обратное FFT

plt.figure(figsize=(12, 8)) # Создаём фигуру размером 12x8 дюймов для двух подграфиков
plt.subplot(2, 1, 1) # Первый подграфик (2 строки, 1 столбец, позиция 1)
plt.plot(freq[:len(freq)//2], np.abs(fft_noisy)[:len(freq)//2], 'b-') # Спектр зашумленного сигнала
plt.title('Спектр зашумленного сигнала') # Заголовок первого подграфика
plt.xlabel('Частота (Гц)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.grid(True) # Сетка

plt.subplot(2, 1, 2) # Второй подграфик (позиция 2)
plt.plot(t, x_noisy_restored.real, 'b-', label='Восстановленный зашумленный сигнал') # Восстановленный сигнал
plt.plot(t, x, 'r--', label='Исходный сигнал') # Исходный сигнал для сравнения
plt.title('Сравнение зашумленного и исходного сигнала') # Заголовок второго подграфика
plt.xlabel('Время (с)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.legend() # Легенда
plt.grid(True) # Сетка
plt.tight_layout() # Регулируем расстояние между подграфиками для аккуратного отображения

# Пункт 2: Анализ прямоугольного сигнала
T = 2 # Период прямоугольного сигнала в секундах
A = 2 # Амплитуда прямоугольного сигнала
t_rect = np.linspace(0, 4, 2000, endpoint=False) # Временной массив от 0 до 4 с с 2000 точками
x_rect = A * (np.mod(t_rect, T) < T/2) # Генерация прямоугольного сигнала (импульсы шириной T/2)

fft_rect = np.fft.fft(x_rect) # Вычисляем спектр прямоугольного сигнала с помощью FFT
dft_rect = DFT_slow(x_rect) # Вычисляем спектр с помощью медленного DFT
freq_rect = np.fft.fftfreq(len(t_rect), t_rect[1] - t_rect[0]) # Частотный массив для прямоугольного сигнала

plt.figure(figsize=(12, 4)) # Новая фигура размером 12x4 дюйма
plt.plot(freq_rect[:len(freq_rect)//2], np.abs(fft_rect)[:len(freq_rect)//2], 'b-', label='FFT') # Спектр FFT
plt.plot(freq_rect[:len(freq_rect)//2], np.abs(dft_rect)[:len(freq_rect)//2], 'r--', label='DFT_slow') # Спектр DFT_slow
plt.title('Спектр прямоугольного сигнала') # Заголовок графика
plt.xlabel('Частота (Гц)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.legend() # Легенда
plt.grid(True) # Сетка

# Добавление шума к прямоугольному сигналу
x_rect_noisy = x_rect + np.random.normal(0, 0.5, x_rect.shape) # Добавляем шум с дисперсией 0.5
fft_rect_noisy = np.fft.fft(x_rect_noisy) # Вычисляем спектр зашумленного прямоугольного сигнала

plt.figure(figsize=(12, 4)) # Новая фигура размером 12x4 дюйма
plt.plot(freq_rect[:len(freq_rect)//2], np.abs(fft_rect_noisy)[:len(freq_rect)//2], 'b-') # Спектр
plt.title('Спектр зашумленного прямоугольного сигнала') # Заголовок графика
plt.xlabel('Частота (Гц)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.grid(True) # Сетка

# Собственная реализация быстрого дискретного преобразования Фурье (БДПФ)
def my_fft(x):
    """Реализация быстрого преобразования Фурье (FFT) с рекурсией"""
    x = np.asarray(x, dtype=complex)  # Преобразуем входной сигнал в массив комплексных чисел
    N = len(x) # Длина сигнала
    # Дополняем сигнал нулями до ближайшей степени двойки для корректной работы алгоритма
    n = int(2 ** np.ceil(np.log2(N))) # Вычисляем ближайшую степень двойки (например, 1024 для 1000)
    if N < n:
        x = np.pad(x, (0, n - N), 'constant') # Добавляем нули в конец сигнала
    N = len(x) # Обновляем длину после дополнения
    if N <= 1: # Базовый случай рекурсии: если длина 1 или меньше, возвращаем сигнал
        return x
    even = my_fft(x[0::2]) # Рекурсивно вычисляем FFT для чётных элементов
    odd = my_fft(x[1::2]) # Рекурсивно вычисляем FFT для нечётных элементов
    T = np.exp(-2j * np.pi * np.arange(N//2) / N) # Вычисляем твиddl-факторы (e^(-j2πk/N))
    return np.concatenate([even + T * odd, even - T * odd]) # Объединяем результаты для полного спектра

# Проверка собственной реализации FFT на сигнале косинуса 50 Гц
t_cos = np.linspace(0, 1, 1000, endpoint=False) # Временной массив для 1 секунды с 1000 точками
x_cos = np.cos(2 * np.pi * 50 * t_cos) # Косинус с частотой 50 Гц
start_time = time.time() # Засекаем время начала выполнения my_fft
my_fft_result = my_fft(x_cos) # Вычисляем спектр с помощью собственной реализации FFT
my_fft_time = time.time() - start_time # Время выполнения my_fft

start_time = time.time() # Засекаем время начала выполнения numpy FFT
fft_result_cos = np.fft.fft(x_cos, n=1024) # Вычисляем спектр с помощью numpy FFT, длина 1024 для согласованности
fft_time_cos = time.time() - start_time # Время выполнения numpy FFT

freq_cos = np.fft.fftfreq(len(my_fft_result), t_cos[1] - t_cos[0]) # Частотный массив для сигнала после my_fft
plt.figure(figsize=(12, 4)) # Новая фигура размером 12x4 дюйма
plt.plot(freq_cos[:len(freq_cos)//2], np.abs(my_fft_result)[:len(freq_cos)//2], 'b-', label='My FFT') # Спектр my_fft
plt.plot(freq_cos[:len(freq_cos)//2], np.abs(fft_result_cos)[:len(freq_cos)//2], 'r--', label='numpy FFT') # Спектр numpy FFT
plt.title('Спектр сигнала косинуса 50 Гц') # Заголовок графика
plt.xlabel('Частота (Гц)') # Подпись оси X
plt.ylabel('Амплитуда') # Подпись оси Y
plt.legend() # Легенда
plt.grid(True) # Сетка
print(f"Время My FFT: {my_fft_time:.6f} сек") # Вывод времени выполнения my_fft
print(f"Время numpy FFT: {fft_time_cos:.6f} сек") # Вывод времени выполнения numpy FFT

plt.show() # Отображаем все созданные графики
