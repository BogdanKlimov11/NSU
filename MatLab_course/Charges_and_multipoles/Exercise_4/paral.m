f = fopen('chargesN.txt', 'rt');
fld = fscanf(f, '%d %d %d %d', [4 4]);
fclose(f);

r = 100;

x = -3:(6/r):3;
y = -1:(5/r):4;
z = -9:(10/r):1;

S1 = 30;
S2 = 60;
S3 = 50;

J = 0;

for i = 1:(length(x)-1)
    for j = 1:(length(y)-1)
        for k = 1:4
            E1 = point_charge_field1(fld(1,k), fld(2:4,k).', [x(i)+6/r  y(j)+5/r -9]);
            E2 = point_charge_field1(fld(1,k), fld(2:4,k).', [x(i)+6/r  y(j)+5/r 1]);
            J = J+(dot(E1, [0 0 -1]) + dot(E2, [0 0 1]))*S1/(r^2);
        end
    end
end

for i = 1:(length(z)-1)
    for j = 1:(length(y)-1)
        for k = 1:4
            E1 = point_charge_field1(fld(1,k), fld(2:4,k).', [-3 y(i)+5/r z(j)+10/r]);
            E2 = point_charge_field1(fld(1,k), fld(2:4,k).', [3 y(i)+5/r z(j)+10/r]);
            
            J = J + (dot(E1, [-1 0 0])+dot(E2, [1 0 0]))*S3/(r^2);
        end
    end
end

for i = 1:(length(x)-1)
    for j = 1:(length(z)-1)
        for k = 1:4  
            E1 = point_charge_field1(fld(1,k), fld(2:4,k).', [x(i)+6/r -1 z(j)+10/r]);
            E2 = point_charge_field1(fld(1,k), fld(2:4,k).', [x(i)+6/r  4 z(j)+10/r]);
            
            J = J+(dot(E1, [0 -1 0])+dot(E2, [0 1 0]))*S2/(r^2);
        end
    end
end

disp(J);