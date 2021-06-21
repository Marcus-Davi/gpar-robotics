clear; close all;clc
% Parametros
freq = 400; % Hz
g = 9.81;


%% Imu inicialização
imu = imuSensor('accel-gyro-mag');
imu.SampleRate = freq;
acc_params = accelparams(...
    'MeasurementRange',2*g,... %2g
    'Resolution',0.1e-3,... %1e-3
    'ConstantBias',0.0,... %0.49
    'NoiseDensity',(400*9.8)*1e-5, ...
    'TemperatureBias',0.294,...
    'TemperatureScaleFactor',0.02,...
    'AxesMisalignment',0,... % 2
    'BiasInstability', 0.1);
imu.Accelerometer = acc_params;

gyr_params = gyroparams(...
    'MeasurementRange',8.7266,... %500º/s
    'Resolution',1.3323e-04,...
    'ConstantBias',0.03491,... %0.03491
    'NoiseDensity',8.7266e-4,...
    'TemperatureBias',0.1,...
    'TemperatureScaleFactor',0.02,...
    'AxesMisalignment',0,... %2
    'AccelerationBias',0.178e-3,...
    'BiasInstability', 0.1);
imu.Gyroscope = gyr_params;

mag_params = magparams( ...
    'MeasurementRange', 100, ...
    'Resolution', 0.1, ...
    'ConstantBias', 0.1, ...
    'AxesMisalignment', 0, ...
    'NoiseDensity', 0.1, ...
    'BiasInstability', 0.1, ...
    'RandomWalk', 0);
imu.Magnetometer = mag_params;

%% Gera dados - sensor parado
samples = 2000;
t = (0:(1/freq):(samples -1)/freq);

acc_lin = zeros(samples,3);
vel_ang = zeros(samples,3);
[acc_data, gyr_data, mag_data] = imu(acc_lin, vel_ang);
csvwrite('parado.csv',[acc_data , gyr_data]);

figure
subplot(3,1,1)
plot(acc_data)
legend('Ax','Ay','Az')
title('Parado')
subplot(3,1,2)
plot(gyr_data)
legend('Gx','Gy','Gz')
subplot(3,1,3)
plot(mag_data)
legend('Mx','My','Mz')

%% Gera dados - pitch pra cima e pra baixo
% Perfil
motion0 = [0 0 0 0 0 0]';
motion1 = [0 0 0 0 pi/4 0]'; % movimento linear
motion2 = [0 0 0 0 -pi/4 0]'; %volta pra origem
motion3 = [0 0 0 0 0 0]'; %volta pra origem
motion4= [0 0 0 -pi/3 0 0]'; %volta pra origem
motion5= [0 0 0 pi/3 0 0]'; %volta pra origem
motion6 = [0 0 0 0 0 0]'; %volta pra origem

[acc_lin0,vel_ang0,orientation0] = ikinematics(motion0,motion1,samples/2,1/freq,0.95);
[acc_lin1,vel_ang1,orientation1] = ikinematics(motion1,motion2,samples/2,1/freq,0.95);
[acc_lin2,vel_ang2,orientation2] = ikinematics(motion2,motion3,samples/2,1/freq,0.95);
[acc_lin3,vel_ang3,orientation3] = ikinematics(motion3,motion4,samples/2,1/freq,0.95);
[acc_lin4,vel_ang4,orientation4] = ikinematics(motion4,motion5,samples/2,1/freq,0.95);
[acc_lin5,vel_ang5,orientation5] = ikinematics(motion5,motion6,samples/2,1/freq,0.95);

acc_lin = [acc_lin0;acc_lin1; acc_lin2;acc_lin3;acc_lin4;acc_lin5];
vel_ang = [vel_ang0;vel_ang1; vel_ang2; vel_ang3; vel_ang4;vel_ang5];
orientation = [orientation0; orientation1; orientation2; orientation3;orientation4;orientation5];



[acc_data, gyr_data, mag_data] = imu(acc_lin, vel_ang,orientation);
csvwrite('movimento.csv',[acc_data , gyr_data,mag_data]);
csvwrite('ground_truth.csv',compact(orientation));


figure
subplot(3,1,1)
plot(acc_data)
legend('Ax','Ay','Az')
title('Movimento Pitch')
subplot(3,1,2)
plot(gyr_data)
legend('Gx','Gy','Gz')
subplot(3,1,3)
plot(mag_data)
legend('Mx','My','Mz')

%% Simple test

pitch = 0;
pitch_acc = 0;
for i =2:length(gyr_data)
    pitch(i) = pitch(i-1) + gyr_data(i,2)/freq;
    pitch_acc(i)  = -atan2(acc_data(i,1), sqrt(acc_data(i,2).*acc_data(i,2) + acc_data(i,3).*acc_data(i,3)));
end

figure
plot(pitch)
hold on
plot(pitch_acc)
legend('Gyro','Acc')



