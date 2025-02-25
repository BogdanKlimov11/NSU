clear;
alpha = 1; beta = .1*0;  % Константы гравитации и сопротивления
dt = 0.05; t = 0;  % Шаг по времени и начальное время
x = 1.5; y = 0; z = 0; R = [x y z];  % Начальные координаты
V = [0 .6 0];  % Начальная скорость
Az = [0 0 .05];  % Ускорение по оси Z

% Настройка подграфиков
subplot(2, 2, 1);
stop = 1;  % Для остановки через интерфейс (при желании раскомментировать строку ниже)
% set(gca ,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, resY/2-resY/4, resX/3, resY/2])

% Графики анимации
h = animatedline(x, y, z, 'color', 'k');  % Траектория
hp = animatedline(x, y, z, 'color', 'b', 'marker', 'o', 'markersize', 5);  % Планета
hS = line(0, 0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);  % Центр поля
h_center = animatedline(0, 0, 0, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);  % Экстремумы
clearpoints(h_center);

% Установка области графика
g = 2; axis([-g g -g g -g g]); axis square; box on;

% Инициализация радиуса-вектора
R = R - V * dt / 2;

% Настройка подграфиков для координат
subplot(2, 2, 2);
grid on; xlabel('t'); ylabel('mean(Xs)');
hx = animatedline(0, 0, 'color', 'k');

subplot(2, 2, 3);
grid on; xlabel('t'); ylabel('mean(Ys)');
hy = animatedline(0, 0, 'color', 'k');

subplot(2, 2, 4);
grid on; xlabel('t'); ylabel('mean(Zs)');
hz = animatedline(0, 0, 'color', 'k');

clearpoints(hx); clearpoints(hy); clearpoints(hz);

% Переменные для хранения координат
r = sqrt(R * R'); r1 = r; r2 = r; Xs = []; Ys = []; Zs = []; i = 0;

% Основной цикл
while ishghandle(h) && stop
    t = t + dt;  % Обновление времени
    clearpoints(hp);  % Очистка старых точек планеты
    R = R + V * dt;  % Обновление радиуса-вектора
    r2 = r1; r1 = r;
    r = sqrt(R * R');
    Xs = [Xs R(1)]; Ys = [Ys R(2)]; Zs = [Zs R(3)];  % Добавление новых координат

    % Проверка экстремума траектории
    if (r2 - r1) * (r1 - r) < 0
        if i == 1
            addpoints(h_center, mean(Xs), mean(Ys), mean(Zs));  % Отметка центра
            addpoints(hx, t, mean(Xs));  % Добавление на график X
            addpoints(hy, t, mean(Ys));  % Добавление на график Y
            addpoints(hz, t, mean(Zs));  % Добавление на график Z
            Xs = []; Ys = []; Zs = []; i = 0;  % Очистка и сброс
        else
            i = i + 1;  % Увеличение счетчика
        end;
    end;
    
    % Вычисление ускорения с учётом гравитации и сопротивления
    A = -alpha * R / r^3 + 2 * beta * R / r^4 + Az;  % Ускорение
    V = V + A * dt;  % Обновление скорости
    x = R(1); y = R(2); z = R(3);

    % Добавление на график планеты
    addpoints(hp, x, y, z);
    drawnow
end;
