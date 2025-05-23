import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fftfreq, fftshift, fft
from scipy.signal import convolve
import librosa

# Настройка стиля графиков
def setup_plot():
    plt.rcParams['font.family'] = 'Arial'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.7
    plt.rcParams['lines.linewidth'] = 1.5

# L2-нормировка вейвлета
def normalize_l2(wavelet):
    return wavelet / np.sqrt(np.sum(np.abs(wavelet) ** 2))

# Вейвлет Морле
def morlet_wavelet(time_array, omega_0=5, alpha=1.0):
    wavelet = np.exp(-time_array ** 2 / alpha ** 2) * np.exp(1j * omega_0 * time_array)
    return normalize_l2(wavelet)

# Спектр вейвлета Морле
def morlet_spectrum(freq_array, omega_0, alpha):
    return alpha * np.sqrt(np.pi) * np.exp(-alpha ** 2 * (freq_array - omega_0) ** 2 / 4)

# Вейвлет Мексиканской шляпы
def mexican_hat_wavelet(time_array, sigma=0.5, t_0=0):
    wavelet = (1 - ((time_array - t_0) / sigma) ** 2) * np.exp(-((time_array - t_0) / sigma) ** 2 / 2)
    return normalize_l2(wavelet)

# Спектр Мексиканской шляпы
def mexican_hat_spectrum(freq_array, omega_0, sigma):
    return (freq_array - omega_0) ** 2 * np.exp(- (freq_array - omega_0) ** 2 / sigma ** 2)

# Вейвлет Хаара
def haar_wavelet(time_array, scale=0.3):
    return np.where((0 <= time_array) & (time_array < scale), 1, np.where((scale <= time_array) & (time_array < 1), -1, 0))

# Задача 1: Вейвлет Морле и его спектр
def task_1():
    print("Plotting Task 1: Morlet wavelet analysis")
    setup_plot()
    time_array = np.linspace(-5, 5, 1000)
    alpha = 1.0
    omega_0_list = [2, 5, 10]
    dt = time_array[1] - time_array[0]
    freq_array = fftshift(fftfreq(len(time_array), dt))

    fig = plt.figure(figsize=(18, 8))
    for i, omega_0 in enumerate(omega_0_list):
        psi_t = morlet_wavelet(time_array, omega_0, alpha)
        ax = fig.add_subplot(2, 3, i + 1)
        ax.plot(time_array, np.real(psi_t), label='Реальная часть')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Вейвлет Морле, ω0={omega_0}')
        ax.legend()
        ax.grid(True)

        psi_omega = morlet_spectrum(freq_array, omega_0, alpha)
        ax = fig.add_subplot(2, 3, i + 4)
        ax.plot(freq_array, np.abs(psi_omega))
        ax.set_xlabel('Частота, Гц')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Спектр вейвлета, ω0={omega_0}')
        ax.set_xlim(-15, 15)
        ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 1: Morlet wavelet analysis')
    plt.savefig('task_1.png')
    plt.show()
    plt.close()

# Задача 2: Вейвлет Мексиканской шляпы и его спектр
def task_2():
    print("Plotting Task 2: Mexican Hat wavelet analysis")
    setup_plot()
    time_array = np.linspace(-5, 5, 1000)
    sigma = 0.5
    t_0 = 0
    omega_0_list = [1, 2, 10]
    dt = time_array[1] - time_array[0]
    freq_array = fftshift(fftfreq(len(time_array), dt))

    fig = plt.figure(figsize=(18, 8))
    for i, omega_0 in enumerate(omega_0_list):
        psi_t = mexican_hat_wavelet(time_array, sigma, t_0) * np.exp(1j * omega_0 * t_0)
        ax = fig.add_subplot(2, 3, i + 1)
        ax.plot(time_array, np.real(psi_t), label='Реальная часть')
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Вейвлет Мексиканской шляпы, ω0={omega_0}')
        ax.legend()
        ax.grid(True)

        psi_omega = mexican_hat_spectrum(freq_array, omega_0, sigma)
        ax = fig.add_subplot(2, 3, i + 4)
        ax.plot(freq_array, np.abs(psi_omega))
        ax.set_xlabel('Частота, Гц')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Спектр вейвлета, ω0={omega_0}')
        ax.set_xlim(-15, 15)
        ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 2: Mexican Hat wavelet analysis')
    plt.savefig('task_2.png')
    plt.show()
    plt.close()

# Задача 3: Вейвлет Хаара и его спектр
def task_3():
    print("Plotting Task 3: Haar wavelet analysis")
    setup_plot()
    time_array = np.linspace(-1, 2, 2000)
    scale_values = [0.5, 1.0, 2.0]
    dt = time_array[1] - time_array[0]
    freq_array = fftshift(fftfreq(len(time_array), dt))

    fig = plt.figure(figsize=(18, 8))
    for i, scale in enumerate(scale_values):
        psi_t = normalize_l2(haar_wavelet(time_array / scale))
        ax = fig.add_subplot(2, 3, i + 1)
        ax.plot(time_array, psi_t)
        ax.set_xlabel('Время, с')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Вейвлет Хаара, a={scale}')
        ax.grid(True)

        psi_omega = fft(psi_t)
        ax = fig.add_subplot(2, 3, i + 4)
        ax.plot(freq_array, np.abs(fftshift(psi_omega)))
        ax.set_xlabel('Частота, Гц')
        ax.set_ylabel('Амплитуда')
        ax.set_title(f'Спектр Хаара, a={scale}')
        ax.set_xlim(-10, 10)
        ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 3: Haar wavelet analysis')
    plt.savefig('task_3.png')
    plt.show()
    plt.close()

# Задача 4: Свертка зашумленного сигнала с вейвлетами
def task_4():
    print("Plotting Task 4: Wavelet convolution with noisy signal")
    setup_plot()
    np.random.seed(42)
    time_array = np.linspace(-5, 5, 1000)
    signal = np.sin(5 * time_array) + 0.3 * np.random.randn(len(time_array))
    signal_clean = np.sin(5 * time_array)
    dt = time_array[1] - time_array[0]
    freq_array = fftfreq(len(time_array), dt)
    positive_mask = freq_array >= 0

    morlet = morlet_wavelet(time_array)
    mexhat = mexican_hat_wavelet(time_array)
    haar = normalize_l2(haar_wavelet(time_array, scale=0.3))

    conv_morlet = convolve(signal, np.real(morlet), mode='same')
    conv_mexhat = convolve(signal, mexhat, mode='same')
    conv_haar = convolve(signal, haar, mode='same')

    freq_signal, spectrum_signal = freq_array[positive_mask], np.abs(fft(signal))[positive_mask]
    freq_signal_clean, spectrum_signal_clean = freq_array[positive_mask], np.abs(fft(signal_clean))[positive_mask]
    signal_fft = fft(signal)
    morlet_fft = fft(morlet)
    mexhat_fft = fft(mexhat)
    haar_fft = fft(haar)

    fig = plt.figure(figsize=(15, 12))
    ax = fig.add_subplot(4, 2, 1)
    ax.plot(time_array, signal)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Зашумленный сигнал')
    ax.grid(True)

    ax = fig.add_subplot(4, 2, 2)
    ax.plot(freq_signal, spectrum_signal, label='С шумом')
    ax.plot(freq_signal_clean, spectrum_signal_clean, label='Чистый')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектры сигналов')
    ax.set_xlim(0, 10)
    ax.legend()
    ax.grid(True)

    conv_result = np.fft.ifft(signal_fft * morlet_fft).real
    freq_conv, spectrum_conv = freq_array[positive_mask], np.abs(fft(conv_result))[positive_mask]
    ax = fig.add_subplot(4, 2, 3)
    ax.plot(time_array, conv_morlet, label='Свертка')
    ax.plot(time_array, signal, 'r', alpha=0.5, label='Сигнал')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Свертка с вейвлетом Морле')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(4, 2, 4)
    ax.plot(freq_conv, spectrum_conv, '--', label='Спектр свертки')
    ax.plot(freq_signal, spectrum_signal, 'r', label='Спектр сигнала')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектр свертки (Морле)')
    ax.set_xlim(0, 10)
    ax.legend()
    ax.grid(True)

    conv_result = np.fft.ifft(signal_fft * mexhat_fft).real
    freq_conv, spectrum_conv = freq_array[positive_mask], np.abs(fft(conv_result))[positive_mask]
    ax = fig.add_subplot(4, 2, 5)
    ax.plot(time_array, conv_mexhat, 'orange', label='Свертка')
    ax.plot(time_array, signal, 'r', alpha=0.5, label='Сигнал')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Свертка с Мексиканской шляпой')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(4, 2, 6)
    ax.plot(freq_conv, spectrum_conv, 'b', label='Спектр свертки')
    ax.plot(freq_signal, spectrum_signal, 'r', label='Спектр сигнала')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектр свертки (Мексиканская шляпа)')
    ax.set_xlim(0, 10)
    ax.legend()
    ax.grid(True)

    conv_result = np.fft.ifft(signal_fft * haar_fft).real
    freq_conv, spectrum_conv = freq_array[positive_mask], np.abs(fft(conv_result))[positive_mask]
    ax = fig.add_subplot(4, 2, 7)
    ax.plot(time_array, conv_haar, 'g', label='Свертка')
    ax.plot(time_array, signal, 'r', alpha=0.5, label='Сигнал')
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Свертка с вейвлетом Хаара')
    ax.legend()
    ax.grid(True)

    ax = fig.add_subplot(4, 2, 8)
    ax.plot(freq_conv, spectrum_conv, 'b', label='Спектр свертки')
    ax.plot(freq_signal, spectrum_signal, 'r', label='Спектр сигнала')
    ax.set_xlabel('Частота, Гц')
    ax.set_ylabel('Амплитуда')
    ax.set_title('Спектр свертки (Хаара)')
    ax.set_xlim(0, 10)
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 4: Wavelet convolution with noisy signal')
    plt.savefig('task_4.png')
    plt.show()
    plt.close()

# Задача 5: Анализ аудиосигнала с вейвлетами
def task_5():
    print("Plotting Task 5: Audio signal wavelet analysis")
    setup_plot()
    try:
        audio_data, sr = librosa.load("D_296Hz.mp3", sr=None, mono=True)
    except FileNotFoundError:
        print("Файл 'D_296Hz.mp3' не найден. Пожалуйста, проверьте путь к файлу.")
        return

    n_fft = 2048
    time_array = np.linspace(0, len(audio_data) / sr, len(audio_data))

    morlet = morlet_wavelet(time_array, omega_0=1, alpha=100)
    mexhat = mexican_hat_wavelet(time_array, sigma=15)
    haar = normalize_l2(haar_wavelet(time_array, scale=0.3))

    conv_morlet = convolve(audio_data, np.real(morlet), mode='same')
    conv_mexhat = convolve(audio_data, mexhat, mode='same')
    conv_haar = convolve(audio_data, haar, mode='same')

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(2, 2, 1)
    ax.specgram(audio_data, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=sr)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма исходного сигнала')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.specgram(conv_morlet, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=sr)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Морле)')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    ax.specgram(conv_mexhat, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=sr)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Мексиканская шляпа)')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    ax.specgram(conv_haar, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=sr)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Хаара)')
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 5: Audio signal wavelet analysis')
    plt.savefig('task_5.png')
    plt.show()
    plt.close()

# Задача 6: Спектрограмма косинусного сигнала
def task_6():
    print("Plotting Task 6: Cosine signal spectrogram")
    setup_plot()
    fs = 1000
    time_array = np.linspace(0, 5, fs * 5)
    signal = np.cos(2 * np.pi * 5 * time_array)
    n_fft = 2048

    morlet = morlet_wavelet(time_array, omega_0=1, alpha=100)
    mexhat = mexican_hat_wavelet(time_array, sigma=15)
    haar = normalize_l2(haar_wavelet(time_array, scale=0.3))

    conv_morlet = convolve(signal, np.real(morlet), mode='same')
    conv_mexhat = convolve(signal, mexhat, mode='same')
    conv_haar = convolve(signal, haar, mode='same')

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(2, 2, 1)
    ax.specgram(signal, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=fs)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма косинусного сигнала')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 2)
    ax.specgram(conv_morlet, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=fs)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Морле)')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 3)
    ax.specgram(conv_mexhat, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=fs)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Мексиканская шляпа)')
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    ax.specgram(conv_haar, NFFT=n_fft, pad_to=1000, mode='magnitude', Fs=fs)
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Частота, Гц')
    ax.set_title('Спектрограмма (Хаара)')
    ax.grid(True)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Task 6: Cosine signal spectrogram')
    plt.savefig('task_6.png')
    plt.show()
    plt.close()

# Выполнение всех задач
if __name__ == "__main__":
    task_1()
    task_2()
    task_3()
    task_4()
    task_5()
    task_6()
