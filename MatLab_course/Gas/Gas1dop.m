clear;
clear global;

set(0, 'units', 'pixels');
res = get(0, 'screensize');
res_x = res(3);
res_y = res(4);
set(gcf, 'position', [res_x / 2 + 2, (res_y - res_y * 0.8) / 2, res_y * 0.8, res_y * 0.8]);

global rad lx ly m n;

n = 30;  % Количество шаров
dt = 0.1;  % Шаг по времени
tmot = 0;  % Текущее время
lx = 200; ly = 200;  % Размеры бильярда
rad(1:n) = 5;  % Радиусы шаров
m(1:n) = 1;  % Массы шаров
g1 = 0.001;  % Ускорение свободного падения
v0 = 100;  % Начальные скорости и координаты из Set_random.m
out = set_random(v0);  % Задаем случайные координаты частиц
x = out(1, :);
y = out(2, :);
vx = out(3, :);
vy = out(4, :);

% Минимальные значения скорости по x и y
vxmin = min(vx);
vymin = min(vy);

% Максимальные значения скорости по x и y
vxmax = max(vx);
vymax = max(vy);

% Максимальные значения скорости
vmin = sqrt(vxmin^2 + vymin^2);
vmax = sqrt(vxmax^2 + vymax^2);

s = subplot(1, 1, 1);
h = animatedline(0, 0, 'color', 'k');
clearpoints(h);
grid on;
t = 0;

step = 100;
N = step;
for n = 100:-1:10
    check = 0;
    rad = []; rad(1:n) = 5;
    m = []; m(1:n) = 1;
    out = set_random(v0);
    x = out(1, :);
    y = out(2, :);
    vx = out(3, :);
    vy = out(4, :);

    while check == 0 && ishghandle(h)
        counter = 0;
        A = [];
        t = 0;
        while ishghandle(h) && counter < N
            t = t + dt;
            counter = counter + 1;
            [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
            A = [A (x(1)^2 + y(1)^2)];
            pause(0.000001);
            timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
            title(s, timelabel);
        end
        % Разворот скорости
        vx = -vx;
        vy = -vy;

        while ishghandle(h) && counter > 1
            t = t + dt;
            counter = counter - 1;
            [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
            delta = (x(1)^2 + y(1)^2) - A(counter);
            if (delta > 0.01)
                delta = delta;
                addpoints(h, n, (N - counter) * dt);
                check = 1;
                break;
            end
            pause(0.000001);
            timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
            title(s, timelabel);
        end

        if check == 0
            N = N + step;  % Увеличиваем количество шагов, пока не получим подходящее значение
        end
    end
end
