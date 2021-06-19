% Error State Kalman Filter

%% Leitura
clear;
clc;
movimento_filename = '../../datasets/simulation/movimento.csv';
parado_filename = '../../datasets/simulation/parado.csv';

data = csvread(movimento_filename);
calib = csvread(parado_filename);

       accx = data(:,1);
       accy = data(:,2);
       accz = data(:,3);

       gyrox = data(:,4);
       gyroy = data(:,5);
       gyroz = data(:,6); 

tam = length(gyrox);

%% Calibração
       
calib_acc = [calib(:,1) calib(:,2) calib(:,3)];
calib_gyro = [calib(:,4) calib(:,5) calib(:,6)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));

gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);

%% Setup
x = [1 0 0 0]'; %--> Nominal State (Only the quaternion)
dx = [1 0 0 0]'; %--> Error State (Only the quaternion)
x_true = [0 0 0 0]'; %--> True State (Only the quaternion)

dt = 1/400;
g = 9.8;

F = eye(4);
P = eye(4);

Q = diag([1 var(calib_gyro)]);
Q(4,4) = 0.00001;
R = diag([1 var(calib_acc)]);
R(4,4) = 1;

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
   
   dx_ = F*dx;
   
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
    dz = y - y_;
  %Update
    H = [ 0   0   0  0
         -q2  q3 -q0 q1   
          q1  q0  q3 q2
          q0 -q1 -q2 q3]*2*g;
    
    K = P_*H'*(H*P_*H'+R)^-1;
    P = (eye(4)-K*H)*P_;
    
    dx = dx_ + K*(dz - dx_);
  %Output
  X_ = [q0 -q1 -q2 -q3
        q1 q0 -q3 q2
        q2 q3 q0 -q1
        q2 -q2 q1 q0];
  x_true = X_*dx;
  output(i,1) = normalize(quaternion(x_true(1),x_true(2),x_true(3),x_true(4)));
end
