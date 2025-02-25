clear;

x = [0.0 1.0 2.0 3.0];  % Начальные координаты
p = [2.0 1.0 0.0 1.0];  % Начальные скорости
t = 0; dt = 0.05;  % Интервал времени

stop = 1;  % Флаг остановки
set(gca,'ButtonDownFcn','stop=0;');  % Обработчик нажатия кнопки
set(0,'units','pixels');  % Получение разрешения экрана
res = get(0,'screensize');  % Размеры экрана
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])  % Позиционирование окна

% Инициализация графиков
for i = 1:4
    subplot(2,2,i)
    axis([-5 5 -5 5], 'equal');
    xlim([-5 5]);
    ylim([-5 5]);
    title(['x = ', num2str(x(i)),'   p = ', num2str(p(i))]);
    grid on;
    xlabel('x');
    ylabel('p = x`(t)');
    h(i) = animatedline(x(i), p(i), 'Color', 'black');  % Создание линий для анимации
end

% Основной цикл обновления графиков
while stop && ishghandle(h(1))  % Если stop активен и окно существует
    t = t + dt;  % Обновление времени
    x = x + p*dt;  % Обновление координат
    p = p - sin(x)*dt;  % Обновление скорости

    % Приведение x к диапазону [-pi, pi]
    I = find(x > pi);
    x(I) = x(I) - 2*pi;
    I = find(x < -pi);
    x(I) = x(I) + 2*pi;
    
    % Обновление точек на графиках
    for i = 1:4
        addpoints(h(i), x(i), p(i));  % Добавление новых точек
    end;
    
    drawnow;  % Обновление графиков
end;
