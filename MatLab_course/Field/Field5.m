clear;
alpha = 1; beta = .1;
dt = 0.05;
x1 = 1.5; y1 = 0; R1 = [x1 y1]; V1 = [0 .6];
x2 = 0; y2 = 4; R2 = [x2 y2]; V2 = [.6 0];

stop = 1; set (gca ,'ButtonDownFcn','stop = 0;');
set(0,'units','pixels'); res = get(0,'screensize'); resX = res(3); resY = res(4);
set(gcf,'position', [resX/2-resX/6, 0, resX/3, resY])

subplot(2,1,1)
h1 = animatedline(x1, y1, 'color', 'k');
hp1 = animatedline(x1, y1, 'color', 'k', 'marker', 'o', 'markersize', 5);
h2 = animatedline(x2, y2, 'color', 'b');
hp2 = animatedline(x2, y2, 'color', 'b', 'marker', 'o', 'markersize', 5);
hS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);
G1 = 15; axis([-G1 G1 -G1 G1]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

subplot(2,1,2)
gp1 = animatedline(0, 0, 'color', 'k', 'marker', 'o', 'markersize', 5);
g2 = animatedline((x2-x1), (y2-y1), 'color', 'b');
gp2 = animatedline((x2-x1), (y2-y1), 'color', 'b', 'marker', 'o', 'markersize', 5);
%gS = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);
G2 = 15; axis([-G2 G2 -G2 G2]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

R1 = R1 - V1*dt/2; R2 = R2 - V2*dt/2;
while ishghandle(h1) && stop
clearpoints(hp1); clearpoints(hp2); clearpoints(gp2);
R1 = R1 + V1*dt; R2 = R2 + V2*dt;
r1 = sqrt(R1*R1'); r2 = sqrt(R2*R2');
A1 = -alpha*R1/r1^3 - 2*beta*R1/abs(r2-r1)^3; % Неправильно
A2 = -alpha*R2/r2^3 - 2*beta*R2/abs(r2-r1)^3; % в field5dop исправлено
V1 = V1 + A1*dt; x1 = R1(1); y1 = R1(2);
V2 = V2 + A2*dt; x2 = R2(1); y2 = R2(2);
addpoints(h1, x1, y1); addpoints(hp1, x1, y1);
addpoints(h2, x2, y2); addpoints(hp2, x2, y2);
addpoints(g2, (x2-x1), (y2-y1)); addpoints(gp2, (x2-x1), (y2-y1));
drawnow
end