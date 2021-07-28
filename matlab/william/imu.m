%{
Resolution Gyroscope: 131 LSB/(°/s)
Resolution Accelerometer: 16384 LSB/(m/s²)

ConstantBias Gyroscope: 0.0347 -5.9434e-04 -0.006
ConstantBias Accelerometer: -0.7778 -0.2629 -0.3785

AxesMisalignment Gyroscope: PESQUISAR
AxesMisalignment Accelerometer: PESQUISAR

TemperatureBias Gyroscope: 
TemperatureScaleFactor Gyroscope:

TemperatureBias Accelerometer: 
TemperatureScaleFactor Accelerometer: 0.02
----------------------------------------------
NoiseDensity Gyroscope: 8.7266e-5
BiasInstability Gyroscope:
RandomWalk Gyroscope:

NoiseDensity Accelerometer: 3.92e-3
BiasInstability Accelerometer:
RandomWalk Accelerometer: 
---------------------------------------------
%} 

%% SETUP
clear;clc;

gyro_p = gyroparams( ...
            'MeasurementRange',4.3633, ...
            'Resolution',1.3323e-04, ...
            'ConstantBias',0.03491, ...
            'AxesMisalignment',2, ...
            'NoiseDensity',8.7266e-4, ...
            'BiasInstability',0.05, ...
            'TemperatureScaleFactor',0.02,...
            'TemperatureBias',0.0349,...
            'AccelerationBias',0.178e-3);

accel_p = accelparams( ...
            'MeasurementRange',19.62, ...
            'Resolution',0.00059875, ...
            'ConstantBias',0.49, ...
            'AxesMisalignment',2, ...
            'NoiseDensity',3.92e-3, ...
            'TemperatureBias',[0.34335 0.34335 0.5886], ...
            'TemperatureScaleFactor',0.0349, ...
             'BiasInstability', 0);
        
IMU = imuSensor('accel-gyro');    
IMU.Gyroscope = gyro_p;
IMU.Accelerometer = accel_p;

%% VARIABLES
Fs = 100; % frequência Hz
N = 1000; %N --> amostras

t = (0:(1/Fs):(N - 1)/Fs)';
%{
Explicação sobre a estrutura do tempo.
Começamos do tempo 0s
O tempo final será a quantidade de amostras - 1 * o tempo por amostra, esse
tempo é 1/Fs. E logo esse 1/Fs também será o incremento do meu vetor tempo.
E por fim, ele está transposto para termos um vetor (N,1)
%}

roll = zeros(N,1); pitch = zeros(N,1); yaw = zeros(N,1);
vel_roll = zeros(N,1); vel_pitch = zeros(N,1); vel_yaw = zeros(N,1);

% Acelerações Lineares
accx = zeros(N,1); accy = zeros(N,1); accz = zeros(N,1);

%% BUILDING THE SIGNAL
for i=1:N
    roll(i) = (pi/2)*sin((2*pi/N)*i);
    vel_roll(i) = (pi^2/N)*sin((2*pi/N)*i); 
end

angVel = [vel_roll vel_pitch vel_yaw];
rotvec = [roll pitch yaw];
orientation = quaternion(rotvec,'rotvec');
acc_linear = [accx accy accz];

[accelReadings , gyroReadings] = IMU(acc_linear,angVel,orientation);

accx = accelReadings(:,1); accy = accelReadings(:,2); accz = accelReadings(:,3);
gyrox = gyroReadings(:,1); gyroy = gyroReadings(:,2); gyroz = gyroReadings(:,3);

acc = [accx accy accz];
gyro = [gyrox gyroy gyroz];

%% SAVING THE DATA

file_data_name = './data/movement_imu.csv';
file_truth_name = './data/true_movement_imu.csv';

movement = [acc gyro];
[q0 q1 q2 q3] = parts(orientation);
ground_truth = [q0 q1 q2 q3];

csvwrite(file_data_name,movement);
csvwrite(file_truth_name,ground_truth);

