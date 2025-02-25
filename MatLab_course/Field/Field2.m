clear;

alpha = 1; beta = .1;  % Константы гравитации и сопротивления
dt = 0.03;  % Шаг по времени
x = 1.5; y = 0; R = [x y];  % Начальные координаты
V = [0 .6];  % Начальная скорость

stop = 1;
set(gca, 'ButtonDownFcn', 'stop = 0;');  % Остановка при нажатии на график
set(0, 'units', 'pixels'); res = get(0, 'screensize');
resX = res(3); resY = res(4);
set(gcf, 'position', [resX / 2 - resX / 6, resY / 2 - resY / 4, resX / 3, resY / 2])

% Настройка графиков
h = animatedline(x, y, 'color', 'k');  % Траектория
hp = animatedline(x, y, 'color', 'b', 'marker', 'o', 'markersize', 5);  % Планета
hS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);  % Центр поля
g = 2; axis([-g g -g g]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

R = R - V * dt / 2;  % Начальная корректировка радиуса-вектора
h_center = animatedline(0, 0, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);  % Экстремумы
h_ogib = animatedline(0, 0, 'color', 'b', 'LineWidth', 2);  % Окружность
clearpoints(h_center); clearpoints(h_ogib);

r = sqrt(R * R'); r1 = r; r2 = r; half = r / 2; Xs = []; Ys = []; i = 0;

% Основной цикл
while ishghandle(h) && stop
    clearpoints(hp);  % Очистка предыдущего положения планеты
    R = R + V * dt;  % Обновление радиуса-вектора
    r2 = r1; r1 = r;
    r = sqrt(R * R');
    Xs = [Xs R(1)]; Ys = [Ys R(2)];

    % Учет экстремума траектории
    if (r2 - r1) * (r1 - r) < 0
        i = i + 1;
        if r > half
            addpoints(h_ogib, x, y);  % Добавление точки на окружности
        elseif r < half && i > 1
            addpoints(h_center, mean(Xs), mean(Ys));  % Отметка центра
            Xs = []; Ys = [];  % Очистка временных массивов
        end;
    end;
    
    % Вычисление ускорения с учетом сопротивления
    A = -alpha * R / r^3 - 2 * beta * R / r^4;  % Ускорение
    V = V + A * dt;  % Обновление скорости
    x = R(1); y = R(2);
    
    % Добавление данных на графики
    addpoints(h, x, y); addpoints(hp, x, y);
    drawnow
end;
