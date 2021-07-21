% Quaternion Kalman Filter

% Yaw muito grande

%% Setup
motion_filename = './data/movement_imu.csv';
%motion_filename = '../../datasets/simulation/movimento.csv';
stationary_filename = '../../data/stationary.csv';
ground_truth_filename = '../../datasets/simulation/ground_truth.csv';

data = csvread(motion_filename);
calib = csvread(stationary_filename);
ground_truth = csvread(ground_truth_filename);

% Accelerometer Data
accx = data(:,1);
accy = data(:,2);
accz = data(:,3);

% Gyroscope Data
gyrox = data(:,4);
gyroy = data(:,5);
gyroz = data(:,6);

size = length(accx);
freq = 100; %frequency Hz
dt = 1/freq; %period s
g = 9.8; %gravity m/s²

%% Calibration

calib_acc = [calib(:,1) calib(:,2) calib(:,3)];
calib_gyro = [calib(:,4) calib(:,5) calib(:,6)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

%%{
accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));

gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);
%}

%% Kalman Filter - Setup
x = [1 0 0 0]; %State Quaternion Vector

F = eye(4);
P = eye(4);
P(4,4) = 0;
clc
Q = diag([1 var(calib_gyro)]);
R = diag(var(calib_acc));

%% Kalman Filter - For
for i=1:size
   q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);  
   wx = gyrox(i); wy = gyroy(i); wz = gyroz(i);
   
    
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
   q0 = x_(1); q1 = x_(2); q2 = x_(3); q3 = x_(4);
   
   y_ = g*[2*q1*q3-2*q0*q2
           2*q0*q1+2*q2*q3
           q0^2-q1^2-q2^2+q3^2];
         
    y = [accx(i) accy(i) accz(i)]';
  
  %Update
    H = 2.*g.*[-q2  q3 -q0 q1   
                q1  q0  q3 q2
                q0 -q1 -q2 q3];
    
    K = P_*H'*(H*P_*H'+R)^-1;
    P = (eye(4)-K*H)*P_;
    
    x = x_ + K*(y - y_);
    
  %Output
  output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
end

%% Graphs

% Por algum motivo, ao usar quat2eul(output,'XYZ') os ângulos ficam
% trocados. Verificar isso com o Marcus.

% With ground truth
%{
euler_kf = quat2eul(output,'XYZ');
euler_true = quat2eul(ground_truth,'XYZ');

subplot(3,1,1)
plot(euler_kf(:,1),'--');
hold on;
plot(euler_true(:,1));
legend('roll','true roll');

subplot(3,1,2)
plot(euler_kf(:,2),'--');
hold on;
plot(euler_true(:,2));
legend('pitch','true pitch');

subplot(3,1,3)
plot(euler_kf(:,3),'--');
hold on;
plot(euler_true(:,3));
legend('yaw','true yaw');
%}

%Without ground truth
%%{
euler_kf = quat2eul(output,'XYZ');

subplot(3,1,1)
plot(euler_kf(:,1),'--');
title('ROLL');

subplot(3,1,2)
plot(euler_kf(:,2),'--');
title('PITCH');

subplot(3,1,3)
plot(euler_kf(:,3),'--');
title('YAW');
%}

%% ROS
%{
rosshutdown % desligar antes
rosinit % roscore

% Ros data
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

    [a b c d] = parts(output(i));
    
    tform.Transform.Rotation.W = a;
    tform.Transform.Rotation.X = b;
    tform.Transform.Rotation.Y = c;
    tform.Transform.Rotation.Z = d;
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
    pause(dt)
end

end
%}

