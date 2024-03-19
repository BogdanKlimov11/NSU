function [x,y,vx,vy]=ballsF8(n,x,y,vx,vy,dt);
global rad lx ly m;    % ������� � ����� ����� � ������� �����.
r=[x; y]; 
v=[vx; vy];    % �������� ������� ��������� � ���������
ldl=[rad; rad];    % ���������� ������ ������� ���� ��������� ������� �����
		% �������, ����������� ������ - ��� ������� ����
lur=[lx-rad ;ly-rad];	% ������ ������� ����
t=dt;       		 % ����������� ����� - ����� �� ������ �� "balls"
while (t>0)
tdl=(ldl-r)./v;%1;         % ������� ������������ �� �������� l � d (�� ��. ����� -
tdl(find(tdl<=0 & v>=0))=inf;         % ����� ������������ � ������� � � ����� ��������)
tur=(lur-r)./v;%1;           % ������������ �� �������� r � u.
tur(find(tur<=0 & v<=0))=inf;     %
	% ����� ������ ������� ������������ �� �������
t1=min(tdl(:)); %
t2=min(tur(:)); %
if(t1<t2)
  t0=t1; j0=find(t0==tdl);
 else
  t0=t2; j0=find(t0==tur);
end; %
% ������� ������ ������������ ����� ���� � ������
x=r(1,:);
y=r(2,:);
vx=v(1,:);
vy=v(2,:);
x0=x(ones(1,n),:)-x(ones(1,n),:)';
y0=y(ones(1,n),:)-y(ones(n,1),:)';
vx0=vx(ones(1,n),:)-vx(ones(n,1),:)';
vy0=vy(ones(1,n),:)-vy(ones(n,1),:)';
rr=x0.*x0+y0.*y0;
vv=vx0.*vx0+vy0.*vy0;
rv=x0.*vx0+y0.*vy0;
d=rv.*rv-(rr-(rad(ones(1,n),:)+rad(ones(n,1),:)').^2).*vv;
tb=-(sqrt(d)+rv)./vv;
tb(find(d<=0 | rv>=0))=inf;
tbm=min(min(tb));
		% ����� ������ ������� ����������,
   if(t<t0)&(t<tbm)
   r=r+v.*t;           % ����� �����
	elseif(t0<=tbm)
      r=r+v.*t0;
      v(j0)=-v(j0);    % � ��������� �������� ��� ����� � ������
   else
      t0=tbm;
      [ii,jj]=find(tb==tbm);
      r=r+v.*t0;

 %     for k=1:length(ii)/2
	    	i=ii(1);
        	j=jj(1);
 
 			r0=r(:,i)-r(:,j);
      	v0=v(:,i)-v(:,j);
      	rv=r0'*v0/((rad(i)+rad(j))*(rad(i)+rad(j)));
      	dv=r0*rv;
      	v(:,i)=v(:,i)-dv*2*m(j)/(m(i)+m(j)); % ��������� ��������� ��� ������������ �����
      	v(:,j)=v(:,j)+dv*2*m(i)/(m(i)+m(j));
      %end %for
   end; %if
   t=t-t0;%-eps;
   end; %while
   x=r(1,:); y=r(2,:); vx=v(1,:); vy=v(2,:);
   % ������� �������� ��������� � �������� 

