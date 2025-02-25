clear;

x = 0.0;  % Начальная координата
p = 1.5;  % Начальная скорость (импульс)
E = p*p/2 + 1 - cos(x);  % Начальная энергия (кинетическая + потенциальная)
t = 0; dt = 0.02;  % Интервал времени

stop = 1;  % Флаг для остановки
set(gca,'ButtonDownFcn','stop=0;');  % Обработчик нажатия для остановки анимации
set(0,'units','pixels');  % Получаем разрешение экрана
res = get(0,'screensize');  % Размеры экрана
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])  % Позиционирование окна

% Подготовка первого графика (x, p)
subplot (2, 1, 1);
axis ([-pi pi -pi pi ], 'equal');  % Ось x от -pi до pi, y от -pi до pi
xlim([-pi pi]); ylim([-pi pi]);
title(['x = ', num2str(x),'   p = ', num2str(p)]);
grid on; xlabel('x'); ylabel('p = x`(t)');
h1 = animatedline(x,p,'Color','black');  % Линия для анимации (x, p)

% Подготовка второго графика (x, E)
subplot (2, 1, 2);
axis ([-pi pi 0.8 1.2], 'equal');  % Ось x от -pi до pi, y от 0.8 до 1.2
xlim([-pi pi]); ylim([0 2]);
title(['x = ', num2str(x),'   E = ', num2str(E)]);
grid on; xlabel('x'); ylabel('E = p*p/2 + 1 - cos(x)');
h2 = animatedline(x,E,'Color','black');  % Линия для анимации (x, E)
h2o = animatedline(x,E,'Color','black','Marker','*','MarkerSize',5);  % Отметка для текущей энергии

% Основной цикл
while stop && ishghandle(h1)  % Пока анимация активна
    t = t + dt;  % Обновляем время
    x = x + p*dt;  % Обновляем координату x
    p = p - sin(x)*dt;  % Обновляем импульс (скорость)
    E = p*p/2 + 1 - cos(x);  % Пересчитываем энергию

    % Обновление графиков
    addpoints(h1, x, p);  % Добавляем точку на график (x, p)
    addpoints(h2, x, E);  % Добавляем точку на график (x, E)
    
    % Обновляем точку, отображающую текущую энергию
    clearpoints(h2o);  % Очистка старых точек
    addpoints(h2o, x, E);  % Добавление новой точки

    % Рисуем обновленные графики
    drawnow;
end;
