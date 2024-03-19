clear
x = 0.0; p = 2.5;
t = 0; dt = 0.01;
h = animatedline(x,p,'Color','black');
ho = animatedline(x,p,'Color','black','Marker','*','MarkerSize',5);
stop = 0;
set (gca ,'ButtonDownFcn','stop=1;');
axis ([-5 5 -5 5], 'equal');
xlim([-5 5]);
ylim([-5 5]);
title('Pendulum');
grid on;
xlabel('x');
ylabel('p = x`(t)');
grid on

x = x - p*dt/2;
while 1
    if stop | ~ishghandle(h) % Если кликнул или закрыл окно
      break
    end
    t = t+dt;
    x = x+p*dt;
    p = p - sin(x)*dt;
    if x > pi
      x = x - 2*pi;
    end;
    if x < -pi
        x=x+2* pi ;
    end;
    addpoints(h,x,p);
    clearpoints(ho);
    addpoints(ho,x,p);
    drawnow;
%    pause(0.1)
end;