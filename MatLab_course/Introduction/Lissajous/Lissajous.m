clear;

% Задание параметров Лиссажу
[a1, a2, w1, w2, fi1, fi2] = inputparam();

% Инициализация времени и шага
t = 0;
dt = 0.03;

% Глобальная переменная для остановки анимации
global stop;

% Начальные координаты для анимации
x = a1 * cos(w1 * t + fi1);
y = a2 * sin(w2 * t + fi2);

% Создание анимационных линий
h = animatedline(x, y, 'Color', 'black');
ho = animatedline(x, y, 'Color', 'r', 'Marker', '*', 'MarkerSize', 5);

% Установка осей и границ
axis([-a1 a1 -a2 a2], 'equal');
title('Lissajous');
grid on;
xlabel('X');
ylabel('Y');

% Инициализация переменной остановки
stop = 1;

% Текущие метки времени и состояния остановки
timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
stoplabel = ['stop = ', num2str(stop)];

% Отображение текста на графике
ht = text(-a1, a2 * 1.05, timelabel);
hs = text(a1, a2 * 1.05, stoplabel);

% Установка обработчика нажатия для переключения состояния анимации
set(gca, 'ButtonDownFcn', 'stop = mod(stop + 1, 2); [t] = animate(t, dt, h, ho, ht, hs);')

% Запуск анимации
[t] = animate(t, dt, h, ho, ht, hs);
