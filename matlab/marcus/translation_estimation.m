clear;close all;clc

movimento_filename = '../../datasets/rosbags/linear_motion.bag';


bag = rosbag(movimento_filename);
imudata = select(bag,'topic','/imu/data');
msgs = readMessages(imudata,'DataFormat','struct');

samples = length(msgs);
ACC = zeros(samples,3);
for i = 1 : samples
    ACC(i,:) = getAcc(msgs{i});


end

% integration
VEL = zeros(samples,3);
POS = zeros(samples,3);
Ts = 0.01;
threshold = 0.3;
% x_ x
A = [1 0;Ts 1];
B = [Ts;Ts^2/2];
X = zeros(samples,2);
x = [0 0]';
for i = 2 : samples
    ax = ACC(i,1);
    ay = ACC(i,2);
    az = ACC(i,2);
       
    
    
    if(ax > threshold || ax < -threshold)
    u = ax;
    else
    u = 0;
    end
    x = A*x + B*u;
    X (i,:) = x;
end



plot(X)
legend('Vel','Pos')
figure
plot(ACC(:,1))



function acc = getAcc(msg)
    acc = [msg.LinearAcceleration.X msg.LinearAcceleration.Y  msg.LinearAcceleration.Z];
    
end