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
function [output] = quaternion_matlab();
clc;
[accx accy accz gyrox gyroy gyroz] = arq_imu(0);
tam = length(gyrox);

%% Correções dos ângulos
[accx_0 accy_0 accz_0 gyrox_0 gyroy_0 gyroz_0] = imu_calibration();

accx = accx + accx_0;
accy = accy + accy_0;
accz = accz + accz_0;

gyrox = gyrox + gyrox_0;
gyroy = gyroy + gyroy_0;
gyroz = gyroz + gyroz_0;

%% Preparação
x = [1 0 0 0]'; %Nosso quatérnio
dt = 1/100;
g = 9.8;

F = eye(4);
P = eye(4);

Q = [0.001 -0.0003 0.0003 0.0003;-0.0003 0.0001 -0.0001 -0.0001;0.0003 -0.0001 0.0001 0.0001;0.0003 -0.0001 0.0001 0.0001];
R = [0.1 0 0 0;0 0.7511 0 0;0 0 0.7759 0;0 0 0 0.8648];


%var(accx) = 0.7511;
%var(accy) = 0.7759;
%var(accz) = 0.8648;

%% Kalman Filter
for i=1:tam
   q0 = x(1);
   q1 = x(2);
   q2 = x(3);
   q3 = x(4);  
   
   wx = gyrox(i);
   wy = gyroy(i);
   wz = gyroz(i);
   
    
  % Prediction
   X = [-q1*wx-q2*wy-q3*wz 
         q0*wx-q3*wy+q2*wz
         q3*wx+q0*wy-q1*wz
        -q2*wx+q1*wy+q0*wz];
   
    F = (dt/2)*[2/dt -wx -wy -wz
                 wx  2/dt wz -wy
                 wy  -wz 2/dt wx
                 wz   wy -wx 2/dt];
    
   x_ = x + (dt/2)*X;
   P_ = F*P*F' + Q;
   
  %Measurement
   q0 = x_(1);
   q1 = x_(2);
   q2 = x_(3);
   q3 = x_(4);
   
   y_ = g*[      0 
           2*q1*q3-2*q0*q2
           2*q0*q1+2*q2*q3
           q0^2-q1^2-q2^2+q3^2];
         
    y = [0 accx(i) accy(i) accz(i)]';
    z = y - y_;
  %Update
    H = [ 0   0   0  0
         -q2  q3 -q0 q1   
          q1  q0  q3 q2
          q0 -q1 -q2 q3]*2*g;
    
    K = P_*H'*(H*P_*H'+R)^-1;
    P = (eye(4)-K*H)*P_;
    
    x = x_ + K*z;
  %Output
  output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
end
    
end