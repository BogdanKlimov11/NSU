clear;
clear global;
cla;
warning off;
global rad lx ly m n;
n=10;	%���������� �����
dt=0.05;             %��� �� �������
tmax=1000.0;             %������������ ����� �����
tmot=0;              %������� �����
lx=200; ly=200;      %������� ���������
rad(1:n)=5;	%������� �����
m(1:n)=1;       %����� �����
g1=0.001;       %��������� ���������� �������
dv=0.2;
%% ��������� �������� � ���������� �� set_random.m
out=set_random(20);  % ������ ��������� ���������� ������, 
x=out(1,:);	      %�������� ���� ���������� �� � � ����� ���������	
y=out(2,:);
vx=out(3,:);
vy=out(4,:);
%% ����������� ������������ � ������������� �������� 
vxmin=min(vx);
vymin=min(vy);
vxmax=max(vx);
vymax=max(vy);
vmin=sqrt(vxmin^2+vymin^2);
vmax=sqrt(vxmax^2+vymax^2);
%% ����� �� ������ 

h=animatedline(x,y); % ������ ����
set(h,'LineStyle','none','color','b','Marker','o','MarkerSize',2.4*rad(1));
axis([0, lx, 0, ly]);
axis('square');

while (tmot<tmax) && ishghandle(h)
     %tmot=tmot+dt;         % 
   %xin=x; yin=y; vxin=vx; vyin=vy; % 
%   tic
 %  Balls_mex(n,dt,g1,x,y,vx,vy,m,rad,[0 0 lx ly]); %����������������� ������ �� � ��� ������� ����� ��������� � ���������
    [x, y, vx, vy]=BallsF8(n,x,y,vx,vy,dt);	%����������������� ������� MATLAB, ����� ���������� ����� ���������
  clearpoints(h);
  addpoints(h,x,y);
    
   
    pause(0.0001);               
 end;
 
