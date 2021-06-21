%Extended Kalman Filter (Quaternion with bias)
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

%% Preparação
x = [1 0 0 0 0 0 0]; 
dt = 1/400;
g = 9.8;

F = eye(7);
P = 0.1*eye(7);
%I have to understand what change in here.
P(7,7) = 0;
P(4,4) = 0;

gyro = [gyrox gyroy gyroz];
a = [accx accy accz];

G = [zeros(4,3);eye(3)]
Q = diag(var(calib_gyro));

%I'm changing the Q(4,4)
%Q = diag([1 var(calib_gyro) dt^2 dt^2 dt^2]);
%Could I multiply for dt^2?


R = diag([1 var(calib_acc)]);
R(4,4) = 1;

%% Kalman Filter
for i=1:tam
   q0 = x(1);
   q1 = x(2);
   q2 = x(3);
   q3 = x(4); 
  wbx = x(5);
  wby = x(6);
  wbz = x(7);
  
  q = [q0;q1;q2;q3];
  wb = [wbx;wby;wbz];
  
  
  wx = gyrox(i);
  wy = gyroy(i);
  wz = gyroz(i);
   
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
     -q2*(wx-wbx)+q1*(wy-wby)+q0*(wz-wbz)];
 
x_ = [q + (dt/2)*X
        wb      ];
    
P_ = F*P*F' + G*Q*G';
%P_ = F*P*F' + Q;

% Measurement
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
% Update
H = g*[ 0     0     0     0   0 0 0
      -2*q2  2*q3 -2*q0  2*q1 0 0 0
       2*q1  2*q0  2*q3  2*q2 0 0 0
       2*q0 -2*q1 -2*q2  2*q3 0 0 0];
   
K = P_*H'*(H*P_*H'+R)^-1;
%P = (eye(7)-K*H)*P_';
 P = (eye(7)-K*H)*P_*(eye(7)-K*H)' + K*R*K';
x = x_ + K*z;

%Output
    output_bias(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
    bias(i,:) = [x(5) x(6) x(7)];
    
end

euler = quat2eul(output_bias);
plot(euler);

output = quaternion_frame();
%% ROS
%%{
rosshutdown % desligar antes
rosinit % roscore

% Ros bias
tftree = rostf;
tform = rosmessage('geometry_msgs/TransformStamped');
tform.ChildFrameId = 'imu_bias';
tform.Header.FrameId = 'map';
tform.Transform.Translation.X = 0;
tform.Transform.Translation.Y = 0;
tform.Transform.Translation.Z = 0;
tform.Transform.Rotation.W = 1;
tform.Transform.Rotation.X = 0;
tform.Transform.Rotation.Y = 0;
tform.Transform.Rotation.Z = 0;

% Ros without bias
%{
tform2 = rosmessage('geometry_msgs/TransformStamped');
tform2.ChildFrameId = 'imu';
tform2.Header.FrameId = 'map';
tform2.Transform.Translation.X = 0.5; % só pra ficar distante e melhorar visualização
tform2.Transform.Translation.Y = 0;
tform2.Transform.Translation.Z = 0;
tform2.Transform.Rotation.W = 1;
tform2.Transform.Rotation.X = 0;
tform2.Transform.Rotation.Y = 0;
tform2.Transform.Rotation.Z = 0;
%}
while(1)  
for i = 1 : tam

    [a b c d] = parts(output_bias(i));
    [q0 q1 q2 q3] = parts(output(i));
    
    
    tform.Transform.Rotation.W = a;
    tform.Transform.Rotation.X = b;
    tform.Transform.Rotation.Y = c;
    tform.Transform.Rotation.Z = d;
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
    
    %{
    tform2.Transform.Rotation.W = q0;
    tform2.Transform.Rotation.X = q1;
    tform2.Transform.Rotation.Y = q2;
    tform2.Transform.Rotation.Z = q3;
    tform2.Header.Stamp = rostime('now');
    sendTransform(tftree,tform2);
    %}
    pause(dt)
end
end
%}