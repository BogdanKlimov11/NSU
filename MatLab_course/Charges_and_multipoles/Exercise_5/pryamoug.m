r0 = [1 2 10];
q = 1;
q1 = -q;
r00 = [0 0 -2];
R = 10;
stepR = 0.2;
Q = 0;
E = 0;
J = 0;
x = -R:stepR:R;
y = -R:stepR:R;
for j = 1:length(y)
        for i = 1:length(x)
            r1 = [x(i)  y(j) 0];
            if (dot(r1,r1))<=R^2           
                E = point_charge_field(q,[r0(1) r0(2) r0(3)], r1) + point_charge_field(q1,[r00(1) r00(2) r00(3)],r1);    
                J = J + dot(E,[0 0 stepR*stepR]);
                E = E*0;
            end;                      
        end;
end;

Q = J/4/pi;
disp(Q);