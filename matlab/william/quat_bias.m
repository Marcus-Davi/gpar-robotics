%Extended Kalman Filter (Quaternion with bias)
%{
State Vector
| q  |
| wb |
Onde
q = q0 q1 q2 q3
wb = 0 wbx wby wbz

x --> 8x1
f --> 8x1
F --> 8x8

h --> 4x1
x --> 8x1
H --> 4x8
%}
%% Leitura
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

wbx = mean_calib_gyro(1,1);
wby = mean_calib_gyro(1,2);
wbz = mean_calib_gyro(1,3);


%% Preparação
x = [1 0 0 0 0 0 0 0]; 
dt = 1/100;
g = 9.8;

F = eye(8);
P = 0.1*eye(8);
P(4,4) = 0;

gyro = [gyrox gyroy gyroz];
a = [accx accy accz];

Q = diag([1 var(calib_gyro) 1 wbx wby wbz]);
Q(4,4) = 0.00001;
R = diag([1 var(calib_acc)]);
R(4,4) = 1;

%% Kalman Filter
for i=1:tam
   q0 = x(1);
   q1 = x(2);
   q2 = x(3);
   q3 = x(4); 
  wbx = x(6);
  wby = x(7);
  wbz = x(8);
  
  q = [q0;q1;q2;q3];
  wb = [0;wbx;wby;wbz];
  
  
  wx = gyrox(i);
  wy = gyroy(i);
  wz = gyroz(i);
   

% Prediction
F = (dt/2)*[2/dt wbx-wx wby-wy wbz-wz 0  q1   q2   q3
            wx-wbx 2/dt wz-wbz wby-wy 0 -q0   q3  -q2
            wy-wby wbz-wz 2/dt wx-wbx 0 -q3  -q0   q1
            wz-wbz wy-wby wbx-wx 2/dt 0  q2  -q1  -q0
              0      0      0     0   0   0    0    0
              0      0      0     0   0  2/dt  0    0
              0      0      0     0   0   0  2/dt   0
              0      0      0     0   0   0    0  2/dt];
          
X = [-q1*(wx-wbx)-q2*(wy-wby)-q3*(wz-wbz) 
      q0*(wx-wbx)-q3*(wy-wby)+q2*(wz-wbz)
      q3*(wx-wbx)+q0*(wy-wby)-q1*(wz-wbz)
     -q2*(wx-wbx)+q1*(wy-wby)+q0*(wz-wbz)];
 
x_ = [q + (dt/2)*X
        wb      ];
    
P_ = F*P*F' + Q;

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
H = g*[ 0     0     0     0   0 0 0 0
      -2*q2  2*q3 -2*q0  2*q1 0 0 0 0
       2*q1  2*q0  2*q3  2*q2 0 0 0 0
       2*q0 -2*q1 -2*q2  2*q3 0 0 0 0];
   
K = P_*H'*(H*P_*H'+R)^-1;
P = (eye(8)-K*H)*P_';
x = x_ + K*z;

%Output
    output(i,1) = normalize(quaternion(x(1),x(2),x(3),x(4)));
    bias(i,:) = [x(6) x(7) x(8)];
    
end

%% ROS
%%{
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