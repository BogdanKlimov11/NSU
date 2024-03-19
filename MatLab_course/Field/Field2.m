clear;
alpha = 1; beta = .1;
dt = 0.03;
x = 1.5; y = 0; R = [x y];
V = [0 .6];

stop = 1;
set (gca ,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize');
resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, resY/2-resY/4, resX/3, resY/2])
h = animatedline(x, y, 'color', 'k'); %Траектория
hp = animatedline(x, y, 'color', 'b', 'marker', 'o', 'markersize', 5); %Планета
hS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7); %Центр поля
g = 2; axis([-g g -g g]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);
R = R - V*dt/2;
h_center = animatedline(0, 0, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);
h_ogib = animatedline(0, 0, 'color', 'b', 'LineWidth', 2);
clearpoints(h_center); clearpoints(h_ogib);

r = sqrt(R*R'); r1 = r; r2 = r; half = r/2; Xs = []; Ys = []; i = 0;
while ishghandle(h) && stop
clearpoints(hp);
R = R + V*dt; %Радиус-вектор
r2 = r1; r1 = r;
r = sqrt(R*R');
Xs = [Xs R(1)]; Ys = [Ys R(2)]; 
if (r2 - r1)*(r1 - r) < 0
    i = i + 1;
    if r > half
        addpoints(h_ogib, x, y);
    elseif r < half && i > 1
        addpoints(h_center, mean(Xs), mean(Ys));
        Xs = []; Ys = [];
    end;
end;
A = -alpha*R/r^3 - 2*beta*R/r^4; %Ускорение
V = V + A*dt; x = R(1); y = R(2);
addpoints(h, x, y); addpoints(hp, x, y);
drawnow
end