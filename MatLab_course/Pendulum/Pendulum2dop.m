clear;
x = 0.0; p = 1.5; E = p*p/2 + 1 - cos(x); Emax = E; Emin = E; dE = 0;
t = 0; dt = 0.01;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3]);

% Настройка графика
axis ([-pi pi 0.8 1.2], 'equal'); 
xlim([0 1.5]); ylim([0 1.5]);
title(['dE/E(dt)']);
grid on; xlabel('dt'); ylabel('dE/E');
h = animatedline('Color','black');

% Процесс для разных значений dt
for dt = 0.01:0.02:1.0
    % Инициализация параметров для нового шага dt
    x = 0.0; p = 1.5; E = p*p/2 + 1 - cos(x); Emax = E; Emin = E;
    
    while stop && ishghandle(h)
        t = t + dt;
        x = x + p*dt;
        p = p - sin(x)*dt;
        E = p*p/2 + 1 - cos(x);  % Пересчитываем энергию
        
        % Обновляем минимальное и максимальное значение энергии
        Emax = max(Emax, E);
        Emin = min(Emin, E);
        
        % Прерывание по условию x
        if x < 0 || x > pi
            break;
        end
        
        % Визуализация данных на графике
        dE = Emax - Emin;
        addpoints(h, dt, dE/E);  % Обновляем точку на графике
        drawnow;  % Рисуем график
    end
end;
