clear;

x = 0.0; p = 2.5;  % Начальные условия
t = 0; dt = 0.01;  % Интервал времени
stop = 0;  % Флаг для остановки

% Инициализация графиков
h = animatedline(x, p, 'Color', 'black');
ho = animatedline(x, p, 'Color', 'black', 'Marker', '*', 'MarkerSize', 5);
axis([-5 5 -5 5], 'equal');
xlim([-5 5]);
ylim([-5 5]);
title('Pendulum');
grid on;
xlabel('x');
ylabel('p = x`(t)');

% Обработчик для остановки анимации
set(gca, 'ButtonDownFcn', 'stop=1;');

% Начальная коррекция
x = x - p * dt / 2;

% Основной цикл анимации
while 1
    if stop || ~ishghandle(h)  % Если кнопка остановки нажата или окно закрыто
        break
    end
    
    t = t + dt;
    
    % Обновление значений переменных
    x = x + p * dt;
    p = p - sin(x) * dt;

    % Приведение x к допустимому диапазону
    if x > pi
        x = x - 2 * pi;
    end
    if x < -pi
        x = x + 2 * pi;
    end

    % Обновление графиков
    addpoints(h, x, p);
    clearpoints(ho);
    addpoints(ho, x, p);

    drawnow;
end;
