function [x, y, vx, vy] = ballsF8 (n, x, y, vx, vy, dt)
    global rad lx ly m;  % Радиусы и массы шаров и размеры стола

    r = [x; y];
    v = [vx; vy];  % Образуем матрицы координат и скоростей
    ldl = [rad; rad];  % Координаты левого нижнего края доступной центрам шаров области, повторенные дважды - для каждого шара
    lur = [lx - rad; ly - rad];  % Правый верхний угол
    t = dt;  % Контрольное время - время до выхода из "balls"

    while (t > 0)
        tdl = (ldl - r) ./ v;  % Моменты столкновения со стенками l и d (но см. далее -
         tdl(find (tdl <= 0 & v >= 0)) = inf;  % Учтем столкновения в прошлом и в очень нескорые)
        tur = (lur - r) ./ v;  % Столкновения со стенками r и u
         tur(find (tur <= 0 & v <= 0)) = inf;  % Учет прошлых и нескорых столкновений

        % Выбор самого раннего столкновения со стенкой
        t1 = min (tdl(:));
        t2 = min (tur(:));
        if (t1 < t2)
            t0 = t1; j0 = find (t0 == tdl);
        else
            t0 = t2; j0 = find (t0 == tur);
        end

        % Находим момент столкновения шаров друг с другом
        x = r(1,:);
        y = r(2,:);
        vx = v(1,:);
        vy = v(2,:);
        x0 = x(ones (1, n), :) - x(ones (1, n), :)';
        y0 = y(ones (1, n), :) - y(ones (n, 1), :)';
        vx0 = vx(ones (1, n), :) - vx(ones (n, 1), :)';
        vy0 = vy(ones (1, n), :) - vy(ones (n, 1), :)';
        rr = x0 .* x0 + y0 .* y0;
        vv = vx0 .* vx0 + vy0 .* vy0;
        rv = x0 .* vx0 + y0 .* vy0;
        d = rv .* rv - (rr - (rad(ones (1, n), :) + rad(ones (n, 1), :)') .^ 2) .* vv;
        tb = -(sqrt (d) + rv) ./ vv;
        tb(find (d <= 0 | rv >= 0)) = inf;
        tbm = min (min (tb));

        % Выбор самого раннего соударения
        if (t < t0) & (t < tbm)
            r = r + v .* t;  % Сдвиг шаров
        elseif (t0 <= tbm)
            r = r + v .* t0;
             v(j0) = -v(j0);  % Изменение скорости при ударе о стенку
        else
            t0 = tbm;
            [ii, jj] = find (tb == tbm);
            r = r + v .* t0;

            i = ii(1);
            j = jj(1);

            r0 = r(:, i) - r(:, j);
            v0 = v(:, i) - v(:, j);
            rv = r0' * v0 / ((rad(i) + rad(j)) * (rad(i) + rad(j)));
            dv = r0 * rv;
            v(:, i) = v(:, i) - dv * 2 * m(j) / (m(i) + m(j));  % Изменение скоростей при столкновении шаров
            v(:, j) = v(:, j) + dv * 2 * m(i) / (m(i) + m(j));
        end

        t = t - t0;  % Обновление оставшегося времени
    end

    x = r(1,:); y = r(2,:); vx = v(1,:); vy = v(2,:);  % Возврат значений координат и скорости
end
