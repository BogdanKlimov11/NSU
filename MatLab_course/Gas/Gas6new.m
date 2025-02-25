clear;
clear global;

set(0, 'units', 'pixels'); 
res = get(0, 'screensize'); 
resX = res(3); resY = res(4);
set(gcf, 'position', [(resX - resY*1.6) + 2, (resY - resY*0.8)/2, resY*1.6, resY*0.8])

global rad lx ly m n;

n = 100;  % Количество частиц
dt = .1;  % Шаг по времени
tmot = 0;  % Текущие время
lx = 200; ly = 200;  % Размеры области
rad(1:n) = 1;  % Радиусы частиц
m(1:n) = 1;  % Массы частиц
g1 = 0.001;  % Константа для силы взаимодействия
v0 = 100;  % Начальная скорость из Set_random.m
out = set_random(v0);  % Генерация случайных координат и скоростей
x = out(1,:);  % Координаты по оси x
y = out(2,:);  % Координаты по оси y
vx = out(3,:);  % Скорости по оси x
vy = out(4,:);  % Скорости по оси y
vxmin = min(vx);  % Минимальная скорость по оси x
vymin = min(vy);  % Минимальная скорость по оси y
vxmax = max(vx);  % Максимальная скорость по оси x
vymax = max(vy);  % Максимальная скорость по оси y
vmin = sqrt(vxmin^2 + vymin^2); % Минимальная скорость
vmax = sqrt(vxmax^2 + vymax^2); % Максимальная скорость

coordsX = [0, lx, lx, 0, 0];
coordsY = [0, 0, ly, ly, 0];

subplot(1, 2, 1)
q = quiver(100, 100, 0, 0, 'Color', 'b', 'LineWidth', 5, 'MaxHeadSize', 10); 
hold on;
h = animatedline(x, y); 
hold off;
set(h, 'LineStyle', 'none', 'color', 'k', 'Marker', 'o', 'MarkerSize', 5 * rad(1));
axis([0, lx, 0, ly]);
axis('square');

s = subplot(1, 2, 2);
E = 0; t = 0;

% Расчёт начальной энергии системы
for i = 1:n
    E = E + vx(i)^2 / 2 + vy(i)^2 / 2;
end;

hE = animatedline(t, E, 'color', 'k');
timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
t_title = title(timelabel);

% Основной цикл моделирования
for j = 1:(10 / dt)
    t = t + dt;
    E = 0;
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
    for i = 1:n
        E = E + (vx(i))^2 / 2 + vy(i)^2 / 2;
    end;
    clearpoints(h); 
    addpoints(h, x, y);
    addpoints(hE, t, E);
    timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
    title(s, timelabel);
    axis(s, [0 t 0 1.5 * E])
    pause(.00001);
end;

set(q, 'Visible', 'on');
while ishghandle(h)
    Ux = sum(vx) / n; 
    Uy = sum(vy) / n; 
    set(q, 'Udata', 5 * Ux, 'Vdata', 5 * Uy);
    vx = vx - Ux; 
    vy = vy - Uy;
    t = t + dt;
    E = 0;
    [x, y, vx, vy] = BallsF8(n, x, y, vx, vy, dt);
    for i = 1:n
        E = E + (vx(i))^2 / 2 + vy(i)^2 / 2;
    end;
    clearpoints(h); 
    addpoints(h, x, y);
    addpoints(hE, t, E);
    timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
    title(s, timelabel);
    axis(s, [0 t 0 10 * E])
    pause(.00001);
end;
