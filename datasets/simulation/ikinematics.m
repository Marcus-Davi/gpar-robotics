function [acc,ang,orientation] = ikinematics(p0,p1,Samples,Ts,alpha)
% Cinemática inversa - gera velocidades / acelerações dados dois pontos no
% espaço. Suavizado por alpha.


offset = round(0.2*Samples);
% Linear
x = linspace(p0(1),p1(1),Samples-2*offset)';
y = linspace(p0(2),p1(2),Samples-2*offset)';
z = linspace(p0(3),p1(3),Samples-2*offset)';

x = [p0(1)*ones(offset,1);x;p1(1)*ones(offset,1)];
y = [p0(2)*ones(offset,1);y;p1(2)*ones(offset,1)];
z = [p0(3)*ones(offset,1);z;p1(3)*ones(offset,1)];

% Angular
x_ = linspace(p0(4),p1(4),Samples-2*offset)';
y_ = linspace(p0(5),p1(5),Samples-2*offset)';
z_ = linspace(p0(6),p1(6),Samples-2*offset)';

x_ = [p0(4)*ones(offset,1);x_;p1(4)*ones(offset,1)];
y_ = [p0(5)*ones(offset,1);y_;p1(5)*ones(offset,1)];
z_ = [p0(6)*ones(offset,1);z_;p1(6)*ones(offset,1)];


orientation = quaternion([x_ y_ z_],'euler','XYZ','frame');

 
% Calcula velocidades
xv = [x(2:end) - x(1:end-1);0]/Ts;
yv = [y(2:end) - y(1:end-1);0]/Ts;
zv = [z(2:end) - z(1:end-1);0]/Ts;

xv_ = [x_(2:end) - x_(1:end-1);0]/Ts;
yv_ = [y_(2:end) - y_(1:end-1);0]/Ts;
zv_ = [z_(2:end) - z_(1:end-1);0]/Ts;

% filtrar velocidades
B = alpha;
A = [1 (alpha-1)];

xv_f = filter(B,A,xv);
yv_f = filter(B,A,yv);
zv_f = filter(B,A,zv);

xv_f_ = filter(B,A,xv_);
yv_f_ = filter(B,A,yv_);
zv_f_ = filter(B,A,zv_);


% calcula acelerações
xa = [xv_f(2:end) - xv_f(1:end-1); 0]/Ts;
ya = [yv_f(2:end) - yv_f(1:end-1); 0]/Ts;
za = [zv_f(2:end) - zv_f(1:end-1); 0]/Ts;






acc = [xa ya za]; %ENU
% acc = [ya xa -za]; %NED

ang = [xv_f_ yv_f_ zv_f_];




end
