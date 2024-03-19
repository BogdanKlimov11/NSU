q = 1; 
r0 = [0 0 1]; 
q_iz = -q; 
r_iz = [0 0 -1];

E = 0; J = 0;

center = [10, 0, 0];
R = 1;

a = 20;
b = 20;

step = 0.1;
x = -a:step:a;
y = -b:step:b;

k = sqrt(pi*R^2/4/a/b);
k0 = sqrt(pi*R^2/4/a/b);
k1 = sqrt(R^2/pi/a/b);

for j = 1:length(y)
    for i = 1:length(x)
        r = [x(i)+center(1)/k1 y(j)+center(2)/k1 0];          
        E = point_charge_field(q,[r0(1) r0(2) r0(3)], r) + point_charge_field(q_iz,[r_iz(1) r_iz(2) r_iz(3)], r);    
        if(R^2 > a*b)
            k = 1;
        end
        J = J + dot(E,[0 0 step^2*k]);
        k = k0;                   
    end
end

disp(J/4/pi);







% q = 1; r0 = [0 0 1];
% q_iz = -q; r_iz = [0 0 1];
% 
% center = [0, 0, 0];
% R = 10;
% a = 20;
% b = 20;
% 
% step = 0.1;
% x = -a:step:a;
% y = -b:step:b;
% 
% Q = 0; E = 0; J = 0;
% 
% S_c = pi*R^2;
% S_s = 4*a*b;
% dS = 4*a*b*step^2;
% 
% for i =1:length(x)
%     for j =1:length(y)
%         r = (x(i)-center(1))^2+(y(j)-center(2))^2;
%         
%         if (R^2 > a*b)
%            %Q = 1-r0(3)*q/(2*pi*sqrt(r0(3)^2+r)^3)*dS;
%            Q = 1;
%         elseif (r <= R^2)   
%            %Q = q-r0(3)*q/(2*pi*sqrt(r0(3)^2+r)^3)*dS*sqrt(S_c/S_s);
%            E = point_charge_field(q, r0, r) + point_charge_field(q_iz, r_iz,r);    
%            J = J + dot(E,[0 0 step*step]);
%            
%         end
%     end
% end
% 
% 
% disp(J/4/pi);