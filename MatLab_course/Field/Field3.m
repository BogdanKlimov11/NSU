clear;

alpha = 1; beta = .1*0;  % Константы гравитации и сопротивления
dt = 0.05;  % Шаг по времени
x = 1.5; y = 0; z = 0; R = [x y z];  % Начальные координаты
V = [0 .6 0];  % Начальная скорость
Az = [0 0 .05];  % Ускорение по оси Z

stop = 1;  % Для остановки через интерфейс (при желании раскомментировать строку ниже)
% set(gca ,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, resY/2-resY/4, resX/3, resY/2])

% Создание графиков
h = animatedline(x, y, z, 'color', 'k');  % Траектория
hp = animatedline(x, y, z, 'color', 'b', 'marker', 'o', 'markersize', 5);  % Планета
hS = line(0, 0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);  % Центр поля
h_center = animatedline(0, 0, 0, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);  % Экстремумы
clearpoints(h_center);

g = 2; axis([-g g -g g -g g]); axis square; box on;
% set(gca, 'xtick', [], 'ytick', [], 'ztick', []);
R = R - V * dt / 2;  % Начальная корректировка радиуса-вектора

% Инициализация переменных
r = sqrt(R * R'); r1 = r; r2 = r; Xs = []; Ys = []; Zs = []; i = 0;

% Основной цикл
while ishghandle(h) && stop
    clearpoints(hp);  % Очистка предыдущего положения планеты
    R = R + V * dt;  % Обновление радиуса-вектора
    r2 = r1; r1 = r;
    r = sqrt(R * R');
    Xs = [Xs R(1)]; Ys = [Ys R(2)]; Zs = [Zs R(3)];

    % Учет экстремума траектории
    if (r2 - r1) * (r1 - r) < 0
        if i == 1
            addpoints(h_center, mean(Xs), mean(Ys), mean(Zs));  % Отметка центра
            Xs = []; Ys = []; Zs = []; i = 0;  % Очистка и сброс
        else
            i = i + 1;  % Увеличение счетчика
        end;
    end;
    
    % Вычисление ускорения с учетом сопротивления и гравитации
    A = -alpha * R / r^3 - 2 * beta * R / r^4 + Az;  % Ускорение
    V = V + A * dt;  % Обновление скорости
    x = R(1); y = R(2); z = R(3);

    % Добавление данных на графики
    addpoints(hp, x, y, z);
    drawnow
end;
