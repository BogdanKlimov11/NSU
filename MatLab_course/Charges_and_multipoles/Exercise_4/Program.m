f = fopen('ChargesN.txt', 'rt');
fld = fscanf(f, '%d %d %d %d', [4, 4]);
fclose(f);

r = 100;

R = 5;
phi = 0:(2*pi/r):(2*pi);
psi = (-pi/2):(pi/r):(pi/2);
center = [0, 0, 0];

J=0;

for i = 1:length(phi)-1
    for j = 1:length(psi)-1
        x = R*cos(phi(i)+2*pi/r/2)*cos(psi(j)+pi/r/2);
        y = R*sin(phi(i)+2*pi/r/2)*cos(psi(j)+pi/r/2);
        z = R*sin(psi(j)+pi/r/2);
        
        dS = 2*pi/r*R*cos(psi(j))*pi/r*R;
        
        for k = 1:4
            E = point_charge_field1(fld(1, k), fld(2:4, k).', [x y z]);
            J = J + dot(E, [x, y, z]/R)*dS;
        end
    end
end

disp(J)
