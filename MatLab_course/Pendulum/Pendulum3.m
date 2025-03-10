clear;

x = 0.0; p = 1.5;
t = 0; dt = 0.02;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3]);

% Настройка первого графика для x и p
subplot(2, 2, 1);
axis([-pi pi -pi pi], 'equal'); xlim([-pi pi]); ylim([-pi pi]);
title(['x = ', num2str(x), '   p = ', num2str(p)]);
grid on; xlabel('p'); ylabel('x = x(p)');
h1 = animatedline(p, x, 'Color', 'black');

% Настройка второго графика для траектории маятника
subplot(2, 2, 2);
axis([-1.2 1.2 -1.2 1.2], 'equal'); xlim([-1.2 1.2]); ylim([-1.2 1.2]);
title(['Pendulum itself']);
grid on; xlabel('X'); ylabel('Y');
h2top = animatedline(-1.2, 0, 'Color', 'black');
h2 = animatedline(0, 0, 'Color', 'black'); 
h2o = animatedline(0, -1.0, 'Color', 'black', 'Marker', 'o', 'MarkerSize', 5);

% Настройка третьего графика для x(t)
subplot(2, 1, 2);
axis([0 50 -pi pi]); xlim([0 50]); ylim([-pi pi]);
title(['x(t)']);
grid on; xlabel('t'); ylabel('x = x(t)');
h3 = animatedline(t, x, 'Color', 'black');

% Основной цикл моделирования маятника
while stop && ishghandle(h1)
    t = t + dt;
    x = x + p*dt;
    p = p - sin(x)*dt;
    
    % Обновление координат X и Y для маятника
    X = sin(x);
    Y = -cos(x);
    
    % Обновление графика x-p
    addpoints(h1, p, x);
    
    % Обновление графика маятника
    clearpoints(h2);
    clearpoints(h2o);
    addpoints(h2, 0, 0); 
    addpoints(h2, X, Y);
    addpoints(h2o, X, Y);
    
    % Обновление графика x(t)
    addpoints(h3, t, x);
    
    drawnow;
end;
