clear;
clear global;

set(0, 'units', 'pixels');
res = get(0, 'screensize');
resX = res(3);
resY = res(4);
set(gcf, 'position', [(resX - resY * 1.6) + 2, (resY - resY * 0.8) / 2, resY * 1.6, resY * 0.8]);

global rad lx ly m n;

n = 100;  % Количество частиц
dt = 0.1;  % Шаг по времени
tmot = 0;  % Текущая временная метка
lx = 200; ly = 200;  % Размеры области

rad(1:n) = 1;  % Радиусы частиц
m(1:n) = 1;  % Массы частиц
g1 = 0.001;  % Коэффициент силы взаимодействия
v0 = 100;  % Начальная скорость

out = set_random(v0);  % Инициализация случайных частиц
x = out(1, :);  % Координаты частиц по оси x
y = out(2, :);  % Координаты частиц по оси y
vx = out(3, :);  % Скорости частиц по оси x
vy = out(4, :);  % Скорости частиц по оси y

vxmin = min(vx);  % Минимальная скорость по x
vymin = min(vy);  % Минимальная скорость по y
vxmax = max(vx);  % Максимальная скорость по x
vymax = max(vy);  % Максимальная скорость по y

vmin = sqrt(vxmin^2 + vymin^2);  % Минимальная скорость
vmax = sqrt(vxmax^2 + vymax^2);  % Максимальная скорость

coordsX = [0, lx, lx, 0, 0];
coordsY = [0, 0, ly, ly, 0];

% Первый subplot — движение частиц
subplot(1, 2, 1);
q = quiver(100, 100, 0, 0, 'Color', 'b', 'LineWidth', 5, 'MaxHeadSize', 10); hold on;
h = animatedline(x, y); hold off;
set(h, 'LineStyle', 'none', 'Color', 'k', 'Marker', 'o', 'MarkerSize', 5 * rad(1));
axis([0, lx, 0, ly]);
axis('square');

% Второй subplot — энергия системы
s = subplot(1, 2, 2);
E = 0;
t = 0;

for i = 1:n
    E = E + (vx(i)^2 + vy(i)^2) / 2;
end;

hE = animatedline(t, E, 'Color', 'k');
timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
t_title = title(timelabel);

T = 5;  % Период обновления
T_counter = 0;  % Счётчик времени для обновлений
counter = 0;

% Основной цикл
while ishghandle(h)
    if (t > 10) && (T_counter >= T)  % Когда прошло 10 секунд
        Ux = sum(vx) / n;  % Средняя скорость по x
        Uy = sum(vy) / n;  % Средняя скорость по y
        
        set(q, 'UData', 5 * Ux, 'VData', 5 * Uy, 'Visible', 'on');
        vx = vx - Ux;  % Коррекция скорости
        vy = vy - Uy;  % Коррекция скорости
        T_counter = 0;  % Сброс счётчика времени
        pause(1);
        set(q, 'Visible', 'off');
    end;
    
    t = t + dt;  % Обновление времени
    T_counter = T_counter + dt;  % Обновление счётчика времени
    E = 0;  % Обновление энергии
    
    % Обновление позиций и скоростей частиц
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
    
    for i = 1:n
        E = E + (vx(i)^2 + vy(i)^2) / 2;
    end;
    
    clearpoints(h);
    addpoints(h, x, y);  % Обновление позиций частиц
    addpoints(hE, t, E);  % Обновление энергии
    timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
    title(s, timelabel);  % Обновление времени на графике
    pause(0.00001);  % Пауза для анимации
end;
