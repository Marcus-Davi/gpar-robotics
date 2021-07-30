% To get data from sensor by ROS 

%% Setup
clear;clc;

ROS_MASTER_IP = '10.0.0.112';
TOPIC = '/mpu_node/mpu_data';
size = 5000;

rosshutdown;
rosinit(ROS_MASTER_IP);

sub = rossubscriber(TOPIC);
msg = rosmessage(sub);

%% Receiving Data

for i = 1: size
   msg = receive(sub);
   
   display(i);
   
   accx(i,1) =msg.LinearAcceleration.X;
   accy(i,1) =msg.LinearAcceleration.Y;
   accz(i,1) =msg.LinearAcceleration.Z;
   gyrox(i,1) =msg.AngularVelocity.X;
   gyroy(i,1) =msg.AngularVelocity.Y;
   gyroz(i,1) =msg.AngularVelocity.Z;
    
end

%% Saving Data
%%{
file_data = [accx accy accz gyrox gyroy gyroz];
file_name = 'movement_roll.csv';

csvwrite(file_name,file_data);
%}
%% Ploting
%%{
subplot(3,2,1);
plot(accx,'--');
title('Accx');

subplot(3,2,3);
plot(accy,'--');
title('Accy');

subplot(3,2,5);
plot(accz,'--');
title('Accz');

subplot(3,2,2);
plot(gyrox,'--');
title('Gyrox');

subplot(3,2,4);
plot(gyroy,'--');
title('Gyroy');

subplot(3,2,6);
plot(gyroz,'--');
title('Gyroz');
%}

