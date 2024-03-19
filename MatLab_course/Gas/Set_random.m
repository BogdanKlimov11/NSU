function out=set_random(v_mean);
% ������ ��������� ���������� ����� ������ ���������. ��������� �� ����������.
% �������� ���� ����� ���������� �� ��� � � ����� ��������� �������.
global rad lx ly n;
vx=v_mean*ones(1,n);
vy=zeros(1,n);
x=zeros(1,n);
y=zeros(1,n);
for i=1:n
   l=0;
   k=0;
   while k<100
      xt=rad(i)+rand*(lx-2*rad(i));
      yt=rad(i)+rand*(ly-2*rad(i));
      l=0;
      for j=1:i-1
      	d=sqrt((xt-x(j))^2+(yt-y(j))^2);
      	if d<rad(i)+rad(j)
         	l=1;
      	end
      end
      if l==0
         break;
      end
      k=k+1;
      if k==99 
         error('����� �����!!! ���������� ��������� �� ���������� ��� ������.');
      end
   end
   x(i)=xt;
   y(i)=yt;
end
out=[x;y;vx;vy];