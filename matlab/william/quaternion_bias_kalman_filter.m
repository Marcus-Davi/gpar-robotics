%Extended Kalman Filter (Quaternion with bias)

%% Setup

clear;clc;
motion_filename = './data/movement_pitch.csv';
%motion_filename = '../../datasets/rosbags/street_imu.csv';
%stationary_filename = '../../datasets/simulation/parado.csv';
stationary_filename = './data/stationary.csv';
%ground_truth_filename = '../../datasets/simulation/ground_truth.csv';
ground_truth_filename = './data/true_movement_imu.csv';

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

% Some important variables
size = length(gyrox);
freq = 100; % frequency Hz
dt = 1/freq; % period s
g = 9.8; %gravity m/s²

%% Calibration
calib_acc = [calib(:,1) calib(:,2) calib(:,3)];
calib_gyro = [calib(:,4) calib(:,5) calib(:,6)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

%{
accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));
%}
%%{
gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);
%}

%% Kalman Filter - Setup
x = [1 0 0 0 0 0 0]; % State Vector (Q,w_bias) 7x1
%Q = (q0,q1,q2,q3)
q0 = 0; q1 = 0; q2 = 0; q3 = 0; 

%w_bias = (wbx,wby,wbz)
wbx = 0; wby = 0; wbz = 0; 

F = eye(7); %Jacobian Matrix of state vector  &f/&x (partial derivation)
P = eye(7); %Estimate Uncertainty
P(4,4) = 0; % relacionado com a confiança na estimativa em yaw
P(7,7) = 0; % relacionado com a confiança do bias no yaw


G = [zeros(4,3);eye(3)];

Q = diag(dt^2*[0.013 0.0136 0.0129]); % Process Noise Matrix 3x3
%Q = diag(dt^2*10^-4*[1.6443 1.1245 2.2742]);

R = diag([0.34335 0.34335 0.5886]);% Measurement Uncertainty 3x3
%R = diag([0.0019 0.0018 0.0039]); 
%% Kalman Filter - For
for i=1:size
   q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
   wbx = x(5); wby = x(6); wbz = x(7);
  
   q = [q0 q1 q2 q3]'; wb = [wbx wby wbz]'
   wx = gyrox(i); wy = gyroy(i); wz = gyroz(i);
   
% Prediction
F = (dt/2)*[2/dt wbx-wx wby-wy wbz-wz   q1   q2   q3
            wx-wbx 2/dt wz-wbz wby-wy  -q0   q3  -q2
            wy-wby wbz-wz 2/dt wx-wbx  -q3  -q0   q1
            wz-wbz wy-wby wbx-wx 2/dt   q2  -q1  -q0
              0      0      0     0     2/dt  0    0
              0      0      0     0      0  2/dt   0
              0      0      0     0      0    0  2/dt];
          
X = [-q1*(wx-wbx)-q2*(wy-wby)-q3*(wz-wbz) 
      q0*(wx-wbx)-q3*(wy-wby)+q2*(wz-wbz)
      q3*(wx-wbx)+q0*(wy-wby)-q1*(wz-wbz)
     -q2*(wx-wbx)+q1*(wy-wby)+q0*(wz-wbz)]; % Quaternion Multiplication
 
x_ = [q + (dt/2)*X
        wb      ];
    
P_ = F*P*F' + G*Q*G';

% Measurement
   q0 = x_(1); q1 = x_(2); q2 = x_(3); q3 = x_(4);
   
   y_ = g*[2*q1*q3-2*q0*q2
           2*q0*q1+2*q2*q3
           q0^2-q1^2-q2^2+q3^2];
         
    y = [accx(i) accy(i) accz(i)]'; %Measurement Data
    
    z = y - y_; 
    
% Update
H = g*[-2*q2  2*q3 -2*q0  2*q1 0 0 0
        2*q1  2*q0  2*q3  2*q2 0 0 0
        2*q0 -2*q1 -2*q2  2*q3 0 0 0];
   
K = P_*H'*(H*P_*H'+R)^-1;

P = (eye(7)-K*H)*P_';
%P = (eye(7)-K*H)*P_*(eye(7)-K*H)' + K*R*K'; %I saw that form is better than the other but I don't see mmuch difference

x = x_ + K*z; %Update State Vector

%Output
    output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4))); %We need to normalize to be able to get the euler angles
    bias(i,:) = [x(5) x(6) x(7)];
end

%% Graphs

    %With ground truth
    %{
    euler = quat2eul(output,'XYZ');
    ground_truth = quat2eul(ground_truth,'XYZ');
    
    subplot(3,1,1);
    plot(euler(:,1),'--');
    hold on;
    plot(ground_truth(:,1),'-');
    legend('Roll','True Roll');
    title('ROLL');
    
    subplot(3,1,2);
    plot(euler(:,2),'--');
    hold on;
    plot(ground_truth(:,2),'-');
    legend('Pitch','True Pitch');
    title('PITCH');
    
    subplot(3,1,3);
    plot(euler(:,3),'--');
    hold on;
    plot(ground_truth(:,3),'-');
    legend('Yaw','True Yaw');
    title('YAW');
    %}
    
    %Without Ground Truth
    %%{
    euler = quat2eul(output,'XYZ');
    subplot(3,1,1);
    plot(euler(:,1),'--');
    title('ROLL');
    
    subplot(3,1,2);
    plot(euler(:,2),'--');
    title('PITCH');
    
    subplot(3,1,3);
    plot(euler(:,3),'--');
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
for i = 1 : size

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
