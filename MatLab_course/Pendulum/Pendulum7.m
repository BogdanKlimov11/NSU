clear;

% Параметры амплитудных колебаний
f = 0.26;  % Амплитуда входящего сигнала
lam = 0.095;  % Параметр коэффициента при изгибе в уравнении для волнового трения
a = 0.05:0.001:1.0; 
aa = a.^2;

% Вычисление d и фильтрация значений
d = 4 * lam^4 - 4 * lam^2 * (1 - 0.5 * aa) + f^2 ./ (4 * aa);
k = find(d >= 0); 
aa = aa(k); 
a = a(k); 
d = d(k);

% Вычисление частот w1 и w2
w1 = sqrt(1 - 2 * lam^2 - 0.5 * aa - sqrt(d));
w2 = sqrt(1 - 2 * lam^2 - 0.5 * aa + sqrt(d));

% Настройка осей и отображение графика
axis([0.6 1 0 2]);
grid on;
xlabel('w');
ylabel('a(w)');

% Создание графиков для теоретических значений
h_teor1 = animatedline(real(w1(1)), a(1), 'color', 'black');
h_teor2 = animatedline(real(w2(1)), a(1), 'color', 'black');

for i = 1:length(w1)
    addpoints(h_teor1, real(w1(i)), 2 * a(i));
end;

for i = 1:length(w2)
    addpoints(h_teor2, real(w2(i)), 2 * a(i));
end;

drawnow;

% Создание линий для отображения данных
h_forwards = animatedline(0, 0, 'LineStyle', 'none', 'color', 'r', 'marker', '.', 'markersize', 5);
h_backwards = animatedline(0, 0, 'LineStyle', 'none', 'color', 'b', 'marker', '.', 'markersize', 5);

clearpoints(h_forwards);
clearpoints(h_backwards);

% Инициализация переменных для расчёта амплитуд
A = 0; 
A2 = 0;
x = 0.0; 
p = 0;
t = 0; 
dt = 0.01;
k = 2 * lam;
w = 0.6; 
dw = 0.001;
phi = 0;
As = []; 
Ws = [];
As2 = []; 
Ws2 = [];

% Цикл для увеличения w
while w < 1 && ishghandle(h_forwards)
    w = w + dw;
    A = 0; 
    A2 = 0;
    x1 = x; 
    x2 = x;
    
    % Цикл для вычисления амплитуд при увеличении w
    while ishghandle(h_forwards)
        x2 = x1;
        x1 = x;
        t = t + dt;
        x = x + p * dt;
        dphi = w * dt;
        phi = phi + dphi;
        
        % Уравнение движения с нелинейной силой трения
        p = p - (x - (x^3) / 6) * dt - k * p * dt + f * sin(phi) * dt;
        
        % Проверка на стабилизацию амплитуды
        if (x2 - x1) * (x1 - x) < 0
            A2 = A;
            A = abs(x1);
        end;
        
        if abs((A2 - A) / A) < 0.0001
            As = [As A];
            Ws = [Ws w];
            addpoints(h_forwards, w, A);
            drawnow;
            break;
        end;
    end;
end;

% Цикл для уменьшения w
w = w + dw / 2;

while w > 0.6 && ishghandle(h_forwards)
    w = w - dw;
    A = 0; 
    A2 = 0;
    x1 = x; 
    x2 = x;
    
    % Цикл для вычисления амплитуд при уменьшении w
    while ishghandle(h_forwards)
        x2 = x1;
        x1 = x;
        t = t + dt;
        x = x + p * dt;
        dphi = w * dt;
        phi = phi + dphi;
        
        % Уравнение движения с нелинейной силой трения
        p = p - (x - (x^3) / 6) * dt - k * p * dt + f * sin(phi) * dt;
        
        % Проверка на стабилизацию амплитуды
        if (x2 - x1) * (x1 - x) < 0
            A2 = A;
            A = abs(x1);
        end;
        
        if abs((A2 - A) / A) < 0.0001
            As2 = [As2 A];
            Ws2 = [Ws2 w];
            addpoints(h_backwards, w, A);
            drawnow;
            break;
        end;
    end;
end;

% plot(w1, 2 * a, 'k', w2, 2 * a, 'k', Ws, As, 'm-*', Ws2, As2, 'b-*'); 
