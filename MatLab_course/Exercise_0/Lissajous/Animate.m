function [t] = animate(t, dt, h, ho, ht, hs)
[a1,a2,w1,w2,fi1,fi2] = inputparam();
global stop;
stoplabel = ['stop = ',num2str(stop)];
set(hs,'String',stoplabel);
while(stop)
    t = t + dt;
    timelabel = ['t = ',num2str(t,'%.2f'),' s'];
    set(ht,'String',timelabel);
    x = a1*cos(w1*t + fi1);
    y = a2*sin(w2*t + fi2);
    addpoints(h,x,y);
    clearpoints(ho);
    addpoints(ho,x,y);
%    pause(0.02)
    drawnow
end
stoplabel = ['stop = ',num2str(stop)];
set(hs,'String',stoplabel);
end