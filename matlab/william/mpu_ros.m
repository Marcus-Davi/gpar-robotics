function [accx, accy, accz, gyrox, gyroy, gyroz] = mpu_ros(ROS_MASTER_IP,TOPIC)
%% SETUP
rosshutdown;
clc;
rosinit(ROS_MASTER_IP)
    
sub = rossubscriber(TOPIC);
msg = rosmessage(sub);

i = 1;

while(i<=1000)
   msg = receive(sub);
   %% Aquisição de dados
   accx(i) = msg.LinearAcceleration.X ;
   accy(i) = msg.LinearAcceleration.Y ;
   accz(i) = msg.LinearAcceleration.Z ;
   
   gyrox(i) = msg.AngularVelocity.X ;
   gyroy(i) = msg.AngularVelocity.Y ;
   gyroz(i) = msg.AngularVelocity.Z ;
     
   i = i + 1
end
end
