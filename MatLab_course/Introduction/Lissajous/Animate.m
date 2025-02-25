function [t] = animate(t, dt, h, ho, ht, hs)
% Анимация движения с использованием входных параметров.
% Обновляет графику с учётом времени и условий остановки.

[a1, a2, w1, w2, fi1, fi2] = inputparam();  % Получаем параметры для анимации

global stop;

% Обновление метки о состоянии остановки
stoplabel = ['stop = ', num2str(stop)];
set(hs, 'String', stoplabel);

while(stop)
    t = t + dt;  % Обновляем время
    timelabel = ['t = ', num2str(t, '%.2f'), ' s'];
    set(ht, 'String', timelabel);  % Обновляем метку времени

    % Вычисляем новые координаты
    x = a1 * cos(w1 * t + fi1);
    y = a2 * sin(w2 * t + fi2);

    % Обновляем анимацию
    addpoints(h, x, y);
    clearpoints(ho);  % Очищаем предыдущие точки
    addpoints(ho, x, y);  % Добавляем новые точки

    drawnow  % Обновление графики в реальном времени
end

% Обновляем метку состояния остановки после завершения цикла
stoplabel = ['stop = ', num2str(stop)];
set(hs, 'String', stoplabel);

end
