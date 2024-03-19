clear;
alpha = 1;
dt = 0.05;
x = 1.5; y = 0; R = [x y];
V = [0 .6];

set(0,'units','pixels'); res = get(0,'screensize');
resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, 0, resX/3, resY])
subplot (2, 1, 1);
h = animatedline(x, y, 'color', 'k'); %Траектория
hp = animatedline(x, y, 'color', 'b', 'marker', 'o', 'markersize', 5); %Планета
hS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7); %Центр поля
h_extr = animatedline(x, y, 'linestyle', 'none', 'color', 'm', 'marker', 'square', 'markersize', 10);
clearpoints(h_extr);
g = 2; axis([-g g -g g]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

E = V(1)^2/2 + V(2)^2/2 - alpha/sqrt(R*R');
subplot (2, 1, 2);
axis([-x x 2*E 0]);
hE = animatedline(x, E, 'color', 'k');
hEo = animatedline(x, E, 'marker', 'o', 'color', 'k');

R = R - V*dt/2;
r = sqrt(R*R'); r1 = r; r2 = r;

while ishghandle(h)
clearpoints(hp); clearpoints(hEo);
r2 = r1;
r1 = r;
R = R + V*dt; %Радиус-вектор
r = sqrt(R*R');
A = -alpha*R/r^3; %Ускорение
V = V + A*dt; x = R(1); y = R(2);
E = V(1)^2/2 + V(2)^2/2 - alpha/r;
if (r2 - r1)*(r1 - r) < 0
    addpoints(h_extr, x, y);
end;
addpoints(h, x, y); addpoints(hp, x, y); addpoints(hE, x, E); addpoints(hEo, x, E);
drawnow
end