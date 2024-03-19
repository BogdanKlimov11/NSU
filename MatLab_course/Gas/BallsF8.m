function [x,y,vx,vy]=ballsF8(n,x,y,vx,vy,dt);
global rad lx ly m;    % Радиусы и массы шаров и размеры стола.
r=[x; y]; 
v=[vx; vy];    % Образуем матрицы координат и скоростей
ldl=[rad; rad];    % Координаты левого нижнего краЯ доступной центрам шаров
		% области, повторенные дважды - длЯ каждого шара
lur=[lx-rad ;ly-rad];	% Правый верхний угол
t=dt;       		 % Контрольное времЯ - времЯ до выхода из "balls"
while (t>0)
tdl=(ldl-r)./v;%1;         % Моменты столкновениЯ со стенками l и d (но см. далее -
tdl(find(tdl<=0 & v>=0))=inf;         % учтем столкновениЯ в прошлом и в очень нескорые)
tur=(lur-r)./v;%1;           % СтолкновениЯ со стенками r и u.
tur(find(tur<=0 & v<=0))=inf;     %
	% Выбор самого раннего столкновениЯ со стенкой
t1=min(tdl(:)); %
t2=min(tur(:)); %
if(t1<t2)
  t0=t1; j0=find(t0==tdl);
 else
  t0=t2; j0=find(t0==tur);
end; %
% находим момент столкновениЯ шаров друг с другом
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
		% Выбор самого раннего соударениЯ,
   if(t<t0)&(t<tbm)
   r=r+v.*t;           % сдвиг шаров
	elseif(t0<=tbm)
      r=r+v.*t0;
      v(j0)=-v(j0);    % и изменение скорости при ударе о стенку
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
      	v(:,i)=v(:,i)-dv*2*m(j)/(m(i)+m(j)); % изменение скоростей при столкновении шаров
      	v(:,j)=v(:,j)+dv*2*m(i)/(m(i)+m(j));
      %end %for
   end; %if
   t=t-t0;%-eps;
   end; %while
   x=r(1,:); y=r(2,:); vx=v(1,:); vy=v(2,:);
   % Возврат значений координат и скорости 

