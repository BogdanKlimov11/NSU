clear;

x = 0.0; 
p = 0;
t = 0; 
dt = 0.05;
k = 2 * 0.095;
F = 0.26; 
w = 0.1; 
dw = 0.1;

stop = 1;
set(gca, 'ButtonDownFcn', 'stop = 0;');
set(0, 'units', 'pixels'); 
res = get(0, 'screensize');
set(gcf, 'position', [res(3) / 2 - res(3) / 6, res(4) / 2 - res(3) / 6, res(3) / 3, res(3) / 3]);

% Подготовка первого графика
subplot(2, 2, 1);
axis([-pi pi -pi pi], 'equal'); 
xlim([-pi pi]); 
ylim([-pi pi]);
title(['x = ', num2str(x), '   p = ', num2str(p)]);
grid on; 
xlabel('p'); 
ylabel('x = x(p)');
h1 = animatedline(p, x, 'Color', 'black');

% Подготовка второго графика
subplot(2, 2, 2);
axis([-1.2 1.2 -1.2 1.2], 'equal'); 
xlim([-1.2 1.2]); 
ylim([-1.2 1.2]);
title('Pendulum itself');
grid on; 
xlabel('X'); 
ylabel('Y');
h2top = animatedline(-1.2, 0, 'Color', 'black'); 
addpoints(h2top, 1.2, 0);
h2 = animatedline(0, 0, 'Color', 'black'); 
addpoints(h2, 0, -1.0);
h2o = animatedline(0, -1.0, 'Color', 'black', 'Marker', 'o', 'MarkerSize', 5);

% Подготовка третьего графика
subplot(2, 1, 2);
axis([0 100 -pi pi]); 
xlim([0 800]); 
ylim([-pi pi]);
title('x(t)');
grid on; 
xlabel('t'); 
ylabel('x = x(t)');
h3 = animatedline(x, t, 'Color', 'black');

% Инициализация переменных
phi = w * t;
A = 0; 
A2 = 0;
x1 = x; 
x2 = x;

% Основной цикл
while stop && ishghandle(h1)
    x2 = x1;
    x1 = x;
    t = t + dt;
    x = x + p * dt; 
    
    % Ограничение угла
    if x > pi
        x = x - 2 * pi;
    end;
    if x < -pi
        x = x + 2 * pi;
    end;
    
    % Проверка на стабилизацию амплитуды
    if (x2 - x1) * (x1 - x) < 0
        A2 = A;
        A = abs(x1);
    end;
    
    if A ~= 0 && abs((A2 - A) / A) < 0.05
        w = w + dw;
        A = 0; 
        A2 = 0;
    end;
    
    % Обновление фазы и расчёт импульса
    phi = phi + w * dt;
    p = p - sin(x) * dt - k * p * dt + F * cos(x) * sin(phi) * dt;
    
    % Обновление графиков
    X = sin(x);
    Y = -cos(x);
    
    addpoints(h1, p, x);
    clearpoints(h2);
    clearpoints(h2o);
    addpoints(h2, 0, 0); 
    addpoints(h2, X, Y);
    addpoints(h2o, X, Y);
    addpoints(h3, t, x);
    
    drawnow;
end;
