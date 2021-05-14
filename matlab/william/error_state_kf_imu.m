function [ik_roll ik_pitch] = error_state_kf_imu();
%% SETUP
clc;
    display('1.Movimento 2.Calibration');
    [accx accy accz gyrox gyroy gyroz] = arq_imu(0);
    [accx_0 accy_0 accz_0 gyrox_0 gyroy_0 gyroz_0] = imu_calibration();

f = 100; %Hz
T = 1/f;
tam = length(accx);

pitch = zeros(tam,1);
roll = zeros(tam,1);

ik_pitch = zeros(tam,1);
k_roll = zeros(tam,1);

%% BIAS

accx = accx + accx_0;
accy = accy + accy_0;
accz = accz + accz_0;

gyrox = gyrox + gyrox_0;
gyroy = gyroy + gyroy_0;
gyroz = gyroz + gyroz_0;

%% Calculando Entradas
pitch = -atan2(accx,sqrt(accz.*accz+accy.*accy));
roll = atan2(accy,sqrt(accz.*accz+accx.*accx));

%% Definindo Matrizes
A = [1 -T 0 0;0 1 0 0; 0 0 1 -T;0 0 0 1];
B = [T 0;0 0;0 T;0 0];

var_q = [0.0035 0;0 0.0037];
Q = B*var_q*B';

H = [1 0 0 0;0 0 1 0];
R = [0.0075 0;0 0.0071];

%% Variáveis
x = [0 0 0 0]'; %Nominal State
x_true = [0 0 0 0]'; % True State
dx = [0 0 0 0]'; % Error State
x_m = [0 0 0 0]'; % Measurement
dz = [0 0]'; % Error Measurement

ik_pitch = zeros(tam,1);
ik_roll = zeros(tam,1);

P = eye(4);
% x --> previous state vector
% x_ --> predicted state vector

%% LOOP
for i = 1:tam   
   
    %Predição
    u = [gyrox(i);gyroy(i)];
    x_ = A*x + B*u;
    dx_ = A*dx;
    P_ = A*P*A' + Q;
   
    
    %Medição
    x_m = [roll(i) 0 pitch(i) 0]';
    dz = H*(x_m - x_);
    
    %Update
    K = P_*H'*(H*P_*H' + R)^-1;
  
    dx = dx_ + K*(dz - H*dx_);
    P = (eye(4)-K*H)*P_*(eye(4)-K*H)' + K*R*K';
    
    x = x_;
    
    %Output
    x_true = x_ + dx;
    
    ik_roll(i,1) = x_true(1,1);
    ik_pitch(i,1) = x_true(3,1);
end

end