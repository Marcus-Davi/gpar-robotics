%ESKF with Euler Angles

% EM PROGRESSO

% É necessário fazer a análise de pequenos sinais do erro para entender a
% dinâmica dele.

%% Leitura
clear;
clc;
movimento_filename = './movimentos/pitch_roll_imu.csv';
parado_filename = './movimentos/parado_imu.csv';

data = csvread(movimento_filename);
calib = csvread(parado_filename);

       accx = data(:,4);
       accy = data(:,5);
       accz = data(:,6);

       gyrox = data(:,1);
       gyroy = data(:,2);
       gyroz = data(:,3); 

tam = length(gyrox);

%% Calibração
       
calib_acc = [calib(:,4) calib(:,5) calib(:,6)];
calib_gyro = [calib(:,1) calib(:,2) calib(:,3)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));

gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);

%% Setup
dt = 1/100;
x = [0 0]'; % Nominal State
dx = [0 0]'; % Error State

A = [1  0  
     0  1];
     
B = [dt 0
     0  dt];
 
pitch = -atan2(accx,sqrt(accz.*accz+accy.*accy)); 
roll = atan2(accy,sqrt(accz.*accz+accx.*accx));

Qn = [dt^2*var(calib_gyro(:,1))
      dt^2*var(calib_gyro(:,2))];
  
P = eye(2);

Rn = [0.0075 0;0 0.0071];

%% ESKF

for i=1:tam
    %% Predict
    u = [gyrox(i)
         gyroy(i)];
     
   x_ = A*x + B*u;
    
   P_ = A*P*A' + Qn;
   
   %% Measurement
   y = [roll(i)
        pitch(i)];
    
   H = eye(2);
   %% Update
   K = P_*H'*(H*P_*H' + Rn)^-1;
   dx = K*(y - H*x_);
   
   P = (eye(2)-K*H)*P_;
   
   x = x_ + dx;
   
   %% Output
   
   output(i,1) = x(1); % Roll
   output(i,2) = x(2); % Pitch
end
