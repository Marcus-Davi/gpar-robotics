function [dx dy] = retirar_g()
clear;
clc;
[cf_pitch,cf_roll,gyro_y,gyro_x,acc_p,acc_r,accx,accy,accz,gyrox,gyroy,gyroz] = complementary_filter();

tam = length(cf_pitch);

vx = zeros(tam,1);
vy = zeros(tam,1);

hvx = zeros(tam,1);
hvy = zeros(tam,1);

dx = zeros(tam,1);
dy = zeros(tam,1);

ax_linear = zeros(tam,1);
ay_linear = zeros(tam,1);

lax_linear = zeros(tam,1);
lay_linear = zeros(tam,1);

hax_linear = zeros(tam,1);
hay_linear = zeros(tam,1);

%Gravidade
gx = zeros(tam,1);
gy = zeros(tam,1);
gz = zeros(tam,1);

a = 0.98;
g = 9.8065;
dt = 1/100;

for i=1:tam
   gx(i) = sin(cf_pitch(i))*cos(cf_roll(i))*g;
   gy(i) = -sin(cf_roll(i))*cos(cf_pitch(i))*g;
   gz(i) = cos(cf_roll(i))*cos(cf_pitch(i))*g; 
end
ax_linear = accx + gx;
ay_linear = accy + gy;
for i=2:tam  
   hax_linear(i) = a*(ax_linear(i)-ax_linear(i-1)) + a*hax_linear(i-1);
   hay_linear(i) = a*(ay_linear(i)-ay_linear(i-1)) + a*hay_linear(i-1); 
end



for i=2:tam
    if(abs(ax_linear(i))>0.5)
       vx(i) = vx(i-1) + (ax_linear(i)+ax_linear(i-1))*dt/2; 
    end
    
    if(abs(hay_linear(i))>0.5)
       vy(i) = vy(i-1) + (hay_linear(i)+hay_linear(i-1))*dt/2; 
    end
end

for i=2:tam
    dx(i) = dx(i-1) + (vx(i)+vx(i-1))*dt/2;
    dy(i) = dy(i-1) + (vy(i)+vy(i-1))*dt/2;
end
figure
plot(vy)



end