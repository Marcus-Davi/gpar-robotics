% Kalman Filter com Quatérnios
%{
x = [q]^T Vou começar trabalhando só com o quatérnio e depois adiciono o
BIAS

q = [cos(ang/2) lsen(ang/2) msen(ang/2) nsen(ang/2)]^T
q(0) = [1 0 0 0 0]^T

w = [0 wx wy wz]^T Medições do giroscópio

f(x) = (1/2)*q*w

x(k+1) = x(k) + f(x(k))*dt
%}
function [x] = quaternion_matlab();
clc;
[accx accy accz gyrox gyroy gyroz] = arq_imu(0);

tam = length(gyrox);
w = quaternion(zeros(tam,1),gyrox,gyroy,gyroz);
q = quaternion(1,0,0,0);
dt = 1/100;
x = quaternion(zeros(tam,1),zeros(tam,1),zeros(tam,1),zeros(tam,1));
x(1) = quaternion(0,0,0,0);

for i=2:tam
x(i) = x(i-1) + ((1/2)*q*w(i))*dt;
end

end
