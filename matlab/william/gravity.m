% Gravidade
% Error State Kalman Filter Quaternion

%% Leitura
clc;
movimento_filename = './movimentos/linear2_test_imu.csv';
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
x = [1 0 0 0 0 0 0]; % Nominal State [q wb]
dx = [0 0 0 0 0 0]; %Error State [dangle dwb]

dt = 1/100; 
g = 9.8;

F = eye(6);
P = zeros(6); %Inialization with 0

Rn = diag(var(calib_acc));
Qn = diag([(dt^2)*var(calib_gyro) dt*0.001 dt*0.001 dt*0.001]); %6x6

%% ESKF
for i=1:tam
    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);
    
    wx = gyrox(i);
    wy = gyroy(i);
    wz = gyroz(i);
    w = [0 wx wy wz];
    
    wbx = x(5);
    wby = x(6);
    wbz = x(7);
    wb = [0 wbx wby wbz];
    
    
    X = [-q1*(wx-wbx)-q2*(wy-wby)-q3*(wz-wbz) 
         q0*(wx-wbx)-q3*(wy-wby)+q2*(wz-wbz)
         q3*(wx-wbx)+q0*(wy-wby)-q1*(wz-wbz)
        -q2*(wx-wbx)+q1*(wy-wby)+q0*(wz-wbz)]; % Quaternion Multiplication q * (wm - wb);
    
    %Predict 
    x_ = [x(1:4)'+(dt/2)*X
               x(5:7)']; %Nominal State
    
    F = [quat2rotm((w-wb)*dt) -dt*eye(3)
              zeros(3)           eye(3)];
    
    P_ = F*P*F' + Qn;
    
    %Measurement
     q0 = x_(1);
     q1 = x_(2);
     q2 = x_(3);
     q3 = x_(4);
     
      
     y_ = g*[2*q1*q3-2*q0*q2
             2*q0*q1+2*q2*q3
             q0^2-q1^2-q2^2+q3^2]
         
     y = [accx(i) accy(i) accz(i)]';
    
    Hx = g*[-2*q2  2*q3 -2*q0 2*q1 0 0 0
             2*q1  2*q0  2*q3 2*q2 0 0 0
             2*q0 -2*q1 -2*q2 2*q3 0 0 0];
       
    Xdx = [-(1/2)*q1 -(1/2)*q2 -(1/2)*q3 0 0 0
            (1/2)*q0 -(1/2)*q3  (1/2)*q2 0 0 0
            (1/2)*q3  (1/2)*q0 -(1/2)*q1 0 0 0
           -(1/2)*q2  (1/2)*q1  (1/2)*q0 0 0 0
                0        0         0     1 0 0
                0        0         0     0 1 0
                0        0         0     0 0 1];
    
            
    H = Hx*Xdx;
    
    K = P_*H'*(H*P_*H' + Rn)^-1;
    
    %Update
    P = (eye(6) - K*H)*P_;
    dx = K*(y - y_);
    
    dX = dx(1);
    dY = dx(2);
    dZ = dx(3);
    
    x(1:4) = [q0-q1*dX/2-q2*dY/2-q3*dZ/2
              q1+q0*dX/2-q3*dY/2+q2*dZ/2
              q2+q3*dX/2+q0*dY/2-q1*dZ/2
              q3-q2*dX/2+q1*dY/2+q0*dZ/2];
    x(5:7) = x_(5:7) + dx(4:6);
    
    output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
end

[q0,q1,q2,q3] = parts(output);

euler_eskf = quat2eul(output,'XYZ');

g_x = 9.81.*(2.*q1.*q3-2.*q0.*q2);
g_y = 9.81.*(2.*q0.*q1+2.*q2.*q3);
g_z = 9.81.*( q0.^2-q1.^2-q2.^2+q3.^2);

g_turned =[g_x g_y g_z];



