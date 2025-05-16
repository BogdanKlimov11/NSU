import numpy as np
import matplotlib.pyplot as plt
import time

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# Медленное дискретное преобразование Фурье
def dft_slow(signal):
    n = len(signal)
    k = np.arange(n).reshape((n, 1))
    e = np.exp(-2j * np.pi * k * np.arange(n) / n)
    return np.dot(e, signal)

# Собственная реализация быстрого дискретного преобразования Фурье
def my_fft(signal):
    signal = np.asarray(signal, dtype=complex)
    n = len(signal)
    # Дополняем сигнал до ближайшей степени двойки
    padded_n = int(2 ** np.ceil(np.log2(n)))
    if n < padded_n:
        signal = np.pad(signal, (0, padded_n - n), 'constant')
    n = len(signal)
    if n <= 1:
        return signal
    even = my_fft(signal[0::2])
    odd = my_fft(signal[1::2])
    t = np.exp(-2j * np.pi * np.arange(n//2) / n)
    return np.concatenate([even + t * odd, even - t * odd])

# Задача 1а: Сравнение спектров с помощью FFT и DFT_slow
def task_1a():
    print("Plotting Task 1a: Spectrum comparison (FFT vs DFT_slow)")
    setup_plot()
    sampling_freq = 1000
    time_array = np.linspace(0, 1, sampling_freq, endpoint=False)
    signal = np.cos(2 * np.pi * 50 * time_array) + np.cos(2 * np.pi * 150 * time_array)
    start_time = time.time()
    fft_result = np.fft.fft(signal)
    fft_time = time.time() - start_time
    start_time = time.time()
    dft_result = dft_slow(signal)
    dft_time = time.time() - start_time
    freq = np.fft.fftfreq(len(time_array), 1/sampling_freq)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], np.abs(fft_result)[:len(freq)//2], 'b-', label='FFT')
    plt.plot(freq[:len(freq)//2], np.abs(dft_result)[:len(freq)//2], 'r--', label='DFT_slow')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Дискретный спектр сигнала (FFT и DFT_slow)')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 1a: Spectrum comparison (FFT vs DFT_slow)')
    plt.savefig('task_1a.png')
    print(f"Время FFT: {fft_time:.6f} сек")
    print(f"Время DFT_slow: {dft_time:.6f} сек")
    plt.show()
    plt.close()

# Задача 1б: Восстановление сигнала с помощью обратного FFT
def task_1b():
    print("Plotting Task 1b: Signal restoration with IFFT")
    setup_plot()
    sampling_freq = 1000
    time_array = np.linspace(0, 1, sampling_freq, endpoint=False)
    signal = np.cos(2 * np.pi * 50 * time_array) + np.cos(2 * np.pi * 150 * time_array)
    fft_result = np.fft.fft(signal)
    signal_restored = np.fft.ifft(fft_result)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(time_array, signal.real, 'b-', label='Исходный сигнал')
    plt.plot(time_array, signal_restored.real, 'r--', label='Восстановленный сигнал')
    plt.xlabel('Время, с')
    plt.ylabel('Амплитуда')
    plt.title('Сравнение исходного и восстановленного сигнала')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 1b: Signal restoration with IFFT')
    plt.savefig('task_1b.png')
    plt.show()
    plt.close()

# Задача 1в: Анализ зашумленного сигнала
def task_1c():
    print("Plotting Task 1c: Noisy signal analysis")
    setup_plot()
    sampling_freq = 1000
    time_array = np.linspace(0, 1, sampling_freq, endpoint=False)
    signal = np.cos(2 * np.pi * 50 * time_array) + np.cos(2 * np.pi * 150 * time_array)
    noise = np.random.normal(0, 1, signal.shape)
    signal_noisy = signal + noise
    fft_noisy = np.fft.fft(signal_noisy)
    signal_noisy_restored = np.fft.ifft(fft_noisy)
    freq = np.fft.fftfreq(len(time_array), 1/sampling_freq)
    fig = plt.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(freq[:len(freq)//2], np.abs(fft_noisy)[:len(freq)//2], 'b-')
    ax1.set_xlabel('Частота, Гц')
    ax1.set_ylabel('Амплитуда')
    ax1.set_title('Спектр зашумленного сигнала')
    ax1.grid(True)
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(time_array, signal_noisy_restored.real, 'b-', label='Восстановленный зашумленный сигнал')
    ax2.plot(time_array, signal, 'r--', label='Исходный сигнал')
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('Амплитуда')
    ax2.set_title('Сравнение зашумленного и исходного сигнала')
    ax2.legend()
    ax2.grid(True)
    fig.canvas.manager.set_window_title('Task 1c: Noisy signal analysis')
    plt.tight_layout()
    plt.savefig('task_1c.png')
    plt.show()
    plt.close()

# Задача 2: Анализ прямоугольного сигнала
def task_2():
    print("Plotting Task 2: Rectangular signal analysis")
    setup_plot()
    period = 2.0
    amplitude = 2.0
    time_array = np.linspace(0, 4, 2000, endpoint=False)
    signal = amplitude * (np.mod(time_array, period) < period / 2)
    fft_result = np.fft.fft(signal)
    dft_result = dft_slow(signal)
    freq = np.fft.fftfreq(len(time_array), time_array[1] - time_array[0])
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], np.abs(fft_result)[:len(freq)//2], 'b-', label='FFT')
    plt.plot(freq[:len(freq)//2], np.abs(dft_result)[:len(freq)//2], 'r--', label='DFT_slow')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр прямоугольного сигнала')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 2: Rectangular signal spectrum')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()
    signal_noisy = signal + np.random.normal(0, 0.5, signal.shape)
    fft_noisy = np.fft.fft(signal_noisy)
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], np.abs(fft_noisy)[:len(freq)//2], 'b-')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр зашумленного прямоугольного сигнала')
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 2: Noisy rectangular signal spectrum')
    plt.savefig('task_2_noisy.png')
    plt.show()
    plt.close()

# Задача 3: Проверка собственной реализации FFT
def task_3():
    print("Plotting Task 3: Custom FFT validation")
    setup_plot()
    time_array = np.linspace(0, 1, 1000, endpoint=False)
    signal = np.cos(2 * np.pi * 50 * time_array)
    start_time = time.time()
    my_fft_result = my_fft(signal)
    my_fft_time = time.time() - start_time
    start_time = time.time()
    fft_result = np.fft.fft(signal, n=1024)
    fft_time = time.time() - start_time
    freq = np.fft.fftfreq(len(my_fft_result), time_array[1] - time_array[0])
    fig = plt.figure(figsize=(12, 4))
    plt.plot(freq[:len(freq)//2], np.abs(my_fft_result)[:len(freq)//2], 'b-', label='My FFT')
    plt.plot(freq[:len(freq)//2], np.abs(fft_result)[:len(freq)//2], 'r--', label='numpy FFT')
    plt.xlabel('Частота, Гц')
    plt.ylabel('Амплитуда')
    plt.title('Спектр сигнала косинуса 50 Гц')
    plt.legend()
    plt.grid(True)
    fig.canvas.manager.set_window_title('Task 3: Custom FFT validation')
    plt.savefig('task_3.png')
    print(f"Время My FFT: {my_fft_time:.6f} сек")
    print(f"Время numpy FFT: {fft_time:.6f} сек")
    plt.show()
    plt.close()

if __name__ == "__main__":
    task_1a()
    task_1b()
    task_1c()
    task_2()
    task_3()