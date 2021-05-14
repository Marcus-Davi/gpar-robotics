function mpu_comp_ros(ROS_MASTER_IP,TOPIC)
%% SETUP
rosshutdown;
clc;
rosinit(ROS_MASTER_IP)
    
sub = rossubscriber(TOPIC);
msg = rosmessage(sub);

while(1)
   msg = receive(sub);
   %% Aquisição de dados
   accx = msg.LinearAcceleration.X ;
   accy = msg.LinearAcceleration.Y ;
   accz = msg.LinearAcceleration.Z ;
   
   gyrox(2) = msg.AngularVelocity.X ;
   gyroy(2) = msg.AngularVelocity.Y ;
   gyroz(2) = msg.AngularVelocity.Z ;
     
end
end
