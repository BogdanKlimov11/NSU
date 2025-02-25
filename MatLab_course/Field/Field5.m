clear;

alpha = 1;
beta = 0.1;
dt = 0.05;
x1 = 1.5; y1 = 0; r1 = [x1 y1]; v1 = [0 0.6];
x2 = 0; y2 = 4; r2 = [x2 y2]; v2 = [0.6 0];

stop = 1; 
set(gca, 'ButtonDownFcn', 'stop = 0;');
set(0, 'units', 'pixels');
res = get(0, 'screensize');
res_x = res(3);
res_y = res(4);
set(gcf, 'position', [res_x / 2 - res_x / 6, 0, res_x / 3, res_y]);

subplot(2, 1, 1);
h1 = animatedline(x1, y1, 'color', 'k');
hp1 = animatedline(x1, y1, 'color', 'k', 'marker', 'o', 'markersize', 5);
h2 = animatedline(x2, y2, 'color', 'b');
hp2 = animatedline(x2, y2, 'color', 'b', 'marker', 'o', 'markersize', 5);
h_s = line(0, 0, 'color', 'r', 'marker', 'pentagram', 'markersize', 7);
g1 = 15; axis([-g1 g1 -g1 g1]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

subplot(2, 1, 2);
gp1 = animatedline(0, 0, 'color', 'k', 'marker', 'o', 'markersize', 5);
g2 = animatedline((x2 - x1), (y2 - y1), 'color', 'b');
gp2 = animatedline((x2 - x1), (y2 - y1), 'color', 'b', 'marker', 'o', 'markersize', 5);
g2_axis = 15; axis([-g2_axis g2_axis -g2_axis g2_axis]); axis square; box on;
set(gca, 'xtick', [], 'ytick', []);

r1 = r1 - v1 * dt / 2;
r2 = r2 - v2 * dt / 2;

while ishghandle(h1) && stop
    clearpoints(hp1);
    clearpoints(hp2);
    clearpoints(gp2);
    
    r1 = r1 + v1 * dt;
    r2 = r2 + v2 * dt;
    
    dist_r1 = sqrt(r1 * r1');
    dist_r2 = sqrt(r2 * r2');
    
    a1 = -alpha * r1 / dist_r1^3 - 2 * beta * r1 / abs(dist_r2 - dist_r1)^3; % Неправильно
    a2 = -alpha * r2 / dist_r2^3 - 2 * beta * r2 / abs(dist_r2 - dist_r1)^3; % В Field5dop исправлено
    
    v1 = v1 + a1 * dt;
    x1 = r1(1);
    y1 = r1(2);
    
    v2 = v2 + a2 * dt;
    x2 = r2(1);
    y2 = r2(2);
    
    addpoints(h1, x1, y1);
    addpoints(hp1, x1, y1);
    addpoints(h2, x2, y2);
    addpoints(hp2, x2, y2);
    addpoints(g2, (x2 - x1), (y2 - y1));
    addpoints(gp2, (x2 - x1), (y2 - y1));
    
    drawnow
end
