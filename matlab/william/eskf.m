% Error State Kalman Filter Quaternion

%% Leitura
clc;
movimento_filename = './data/movement_pitch.csv';
%movimento_filename = '../../datasets/simulation/movimento.csv';

%stationary_filename = '../../datasets/simulation/parado.csv';
stationary_filename = './data/stationary.csv';
%ground_truth_filename = '../../datasets/simulation/ground_truth.csv';
ground_truth_filename = './data/true_movement_imu.csv';

data = csvread(movimento_filename);
calib = csvread(stationary_filename);
ground_truth = csvread(ground_truth_filename);

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

%{
accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));

gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);
%}

%% Setup
x = [1 0 0 0 0 0 0]; % Nominal State [q wb]
dx = [0 0 0 0 0 0]; %Error State [dangle dwb]

dt = 1/100; 
g = 9.8;

F = eye(6);
P = zeros(6); %Inialization with 0


Rn = diag(var(calib_acc));
Qn = diag([dt^2*var(calib_gyro) dt*[0.001 0.001 0.001]]);%6x6



%% ESKF
for i=1:tam
    q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
    wx = gyrox(i); wy = gyroy(i); wz = gyroz(i); w = [0 wx wy wz];
    wbx = x(5); wby = x(6); wbz = x(7); wb = [0 wbx wby wbz];
    
    
    X = [-q1*(wx-wbx)-q2*(wy-wby)-q3*(wz-wbz) 
         q0*(wx-wbx)-q3*(wy-wby)+q2*(wz-wbz)
         q3*(wx-wbx)+q0*(wy-wby)-q1*(wz-wbz)
        -q2*(wx-wbx)+q1*(wy-wby)+q0*(wz-wbz)]; % Quaternion Multiplication q * (wm - wb);
    
    %Predict 
    x_ = [x(1:4)'+(dt/2)*X
               x(5:7)']; %Nominal State
    
    F = [quat2rotm((w-wb)*dt) -dt*eye(3)
              zeros(3)           eye(3)];
    
    P_ = F*P*F' + Qn
    
    %Measurement
     q0 = x_(1); q1 = x_(2); q2 = x_(3); q3 = x_(4);
     
      
     y_ = g*[2*q1*q3-2*q0*q2
             2*q0*q1+2*q2*q3
             q0^2-q1^2-q2^2+q3^2];
         
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
    
    dX = dx(1); dY = dx(2); dZ = dx(3);
    
    x(1:4) = [q0-q1*dX/2-q2*dY/2-q3*dZ/2
              q1+q0*dX/2-q3*dY/2+q2*dZ/2
              q2+q3*dX/2+q0*dY/2-q1*dZ/2
              q3-q2*dX/2+q1*dY/2+q0*dZ/2];
    x(5:7) = x_(5:7) + dx(4:6);
    
 
    output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
end

%% Visualization

% With ground truth
%{
euler_eskf = quat2eul(output,'XYZ');
euler_true = quat2eul(ground_truth,'XYZ');

subplot(3,1,1)
plot(euler_eskf(:,1),'--');
hold on;
plot(euler_true(:,1));
legend('roll','true roll');

subplot(3,1,2)
plot(euler_eskf(:,2),'--');
hold on;
plot(euler_true(:,2));
legend('pitch','true pitch');

subplot(3,1,3)
plot(euler_eskf(:,3),'--');
hold on;
plot(euler_true(:,3));
legend('yaw','true yaw');
%}

%Without ground truth
%%{
euler_eskf = quat2eul(output,'XYZ');

subplot(3,1,1)
plot(euler_eskf(:,1),'--');
title('ROLL');

subplot(3,1,2)
plot(euler_eskf(:,2),'--');
title('PITCH');

subplot(3,1,3)
plot(euler_eskf(:,3),'--');
title('YAW');
%}



%% ROS
%{
rosshutdown % desligar antes
rosinit % roscore

% Ros bias
tftree = rostf;
tform = rosmessage('geometry_msgs/TransformStamped');
tform.ChildFrameId = 'imu';
tform.Header.FrameId = 'map';
tform.Transform.Translation.X = 0;
tform.Transform.Translation.Y = 0;
tform.Transform.Translation.Z = 0;
tform.Transform.Rotation.W = 1;
tform.Transform.Rotation.X = 0;
tform.Transform.Rotation.Y = 0;
tform.Transform.Rotation.Z = 0;



while(1)  
for i = 1 : tam

    [q0 q1 q2 q3] = parts(output(i));
       
    tform.Transform.Rotation.W = q0;
    tform.Transform.Rotation.X = q1;
    tform.Transform.Rotation.Y = q2;
    tform.Transform.Rotation.Z = q3;
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
   
    pause(dt)
end
end
%}
