clear;close all;clc
bag = rosbag('street_imu.bag');
message = select(bag,'topic','/imu/data');
imu_data = readMessages(message);

data = zeros(length(imu_data),6);
time = zeros(length(imu_data),1);
for i = 1 : length(imu_data)
    data(i,:) = [imu_data{i}.LinearAcceleration.X imu_data{i}.LinearAcceleration.Y imu_data{i}.LinearAcceleration.Z ...
                 imu_data{i}.AngularVelocity.X imu_data{i}.AngularVelocity.Y imu_data{i}.AngularVelocity.Z ];
             
    time(i) = imu_data{i}.Header.Stamp.seconds;
    
end

Ts = mean(diff(time));
csvwrite('street_imu.csv',data);