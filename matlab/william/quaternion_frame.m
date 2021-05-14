% quaternion frame

%% Leitura
addpath('./movimentos');

movimento = "pitch_imu.csv"; % Colocamos o nome do arquivo que queremos ler

data = csvread(movimento);
       accx = data(:,4);
       accy = data(:,5);
       accz = data(:,6);

       gyrox = data(:,1);
       gyroy = data(:,2);
       gyroz = data(:,3); 

tam = length(gyrox);

%% Calibração
       
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

Q = [0.0001 -0.0003 0.0003 0.0003;-0.0003 0.0001 -0.0001 -0.0001;0.0003 -0.0001 0.0001 0.0001;0.0003 -0.0001 0.0001 0.0001];
R = [1 0 0 0;0 0.7511 0 0;0 0 0.7759 0;0 0 0 0.8648];

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

%% ROS
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
    tform.Header.Stamp = rostime('now')
    sendTransform(tftree,tform)
    pause(dt)
end
end