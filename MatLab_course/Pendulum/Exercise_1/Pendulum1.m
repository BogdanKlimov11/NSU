clear
x = [0.0 1.0 2.0 3.0]; p = [2.0 1.0 0.0 1.0];
t = 0; dt = 0.05;

stop = 1;
set(gca,'ButtonDownFcn','stop=0;');
set(0,'units','pixels'); res = get(0,'screensize');
set(gcf,'position', [res(3)/2-res(3)/6, res(4)/2-res(3)/6, res(3)/3, res(3)/3])

for i = 1:4
    subplot(2,2,i)
    axis ([-5 5 -5 5], 'equal'); xlim([-5 5]); ylim([-5 5]);
    title(['x = ', num2str(x(i)),'   p = ', num2str(p(i))]);
    grid on; xlabel('x'); ylabel('p = x`(t)');
    h(i) = animatedline(x(i),p(i),'Color','black');
end;

%x = x - p*dt/2;
while stop && ishghandle(h(1))
    t = t + dt;
    x = x + p*dt;
    p = p - sin(x)*dt;
    I = find(x > pi); x(I) = x(I) - 2*pi;
    I = find(x < -pi); x(I) = x(I) + 2*pi;
    for i = 1:4
        addpoints(h(i),x(i),p(i));
    end;
    drawnow;
%    pause(0.1)
end;