%-----------------------------------------------%
% Simulador de IMU                              %                  
%  Simular o funcionamento do mpu6050           %
% Luan Amaral                                   % 
%                                               %
%-----------------------------------------------%
clc, clear, close all
addpath('./filtros', './movimentos')

%% constantes
f = 100; %Hz
dt = 1/f;
g = 9.8056;
acc_bias = 0.49;
acc_var = 0.7778;

gyr_bias = 0.03491;
gyr_var  = 1.817313845456145e-04; %dado experimentalmente
%% IMU init
imu = imuSensor('accel-gyro');
imu.SampleRate = f;
acc_params = accelparams(...
     'MeasurementRange',2*g,...
     'Resolution',0.598e-3,...
     'ConstantBias',0.49,...
     'NoiseDensity',3920e-6,...
     'TemperatureBias',0.294,...
     'TemperatureScaleFactor',0.02,...
     'AxesMisalignment',2,...
     'BiasInstability', 0.7778);
 imu.Accelerometer = acc_params;
 
 gyr_params = gyroparams(...
     'MeasurementRange',8.7266,... %500º/s
     'Resolution',1.3323e-04,...
     'ConstantBias',0.03491,...
     'NoiseDensity',8.7266e-4,...
     'TemperatureBias',0.349,...
     'TemperatureScaleFactor',0.02,...
     'AxesMisalignment',2,...
     'AccelerationBias',0.178e-3,...
     'BiasInstability', 0.05);
 imu.Gyroscope = gyr_params;

 %% leitura
 samples = 1000;
 t = (0:(1/f):(samples -1)/f);
 
 [acc_lin, vel_ang, rotvec] = angular_cos(f, samples, 1);
 orientation = quaternion(rotvec, 'rotvec');
 
 [acc_data, gyr_data] = imu(acc_lin, vel_ang, orientation);

 %% calculando os angulos
 roll_acc  = atan2(acc_data(:,2), sqrt(acc_data(:,1).*acc_data(:,1) + acc_data(:,3).*acc_data(:,3)));
 pitch_acc = -atan2(acc_data(:,1), sqrt(acc_data(:,2).*acc_data(:,2) + acc_data(:,3).*acc_data(:,3)));
 
 roll_gyr = zeros(samples,1);
 roll_gyr(1) = rotvec(1,1);
 pitch_gyr = zeros(samples,1);
 pitch_gyr(1) = rotvec(1,2);
 for i=2:samples
     roll_gyr(i) = roll_gyr(i-1) + dt*gyr_data(i,1);
     pitch_gyr(i) = pitch_gyr(i-1) + dt*gyr_data(i,2);
 end

 %% Kalman Filter & Comp filter
 kf = kalmanFilter(dt, gyr_bias, gyr_bias, acc_var, acc_var);
 ikf = indirectKalmanFilter(dt, gyr_bias, gyr_bias, acc_var, acc_var, gyr_bias, gyr_bias);

 roll_kf = zeros(samples, 1);
 pitch_kf = zeros(samples,1);
 roll_ikf = zeros(samples, 1);
 pitch_ikf = zeros(samples,1);
 
 roll_com = zeros(samples, 1);
 pitch_com = zeros(samples,1);
 roll_com(1) = rotvec(1,1);
 roll_kf(1) = rotvec(1,1);
 roll_ikf(1)= rotvec(1,1);
 pitch_com(1) = rotvec(1,2);
 pitch_kf(1) =rotvec(1,2);
 pitch_ikf(1) = rotvec(1,2);
 
 kf.x_kf = [roll_kf(1); gyr_bias; pitch_kf(1); gyr_bias];
 
 for i=2:samples
    %complementar
    roll_com(i) = filtroComplementar(roll_acc(i), dt*gyr_data(i,1), roll_com(i-1));
    pitch_com(i) = filtroComplementar(pitch_acc(i), dt*gyr_data(i,2), pitch_com(i-1));
    
    %kalman filter
    kf.predict(gyr_data(i,1), gyr_data(i,2));
    x = kf.update(roll_acc(i), pitch_acc(i));
    roll_kf(i) = x(1);
    pitch_kf(i) = x(3);
    
    %indirect kalman filter
    ikf.predict(gyr_data(i,1), gyr_data(i,2) );
    pos = ikf.update(roll_acc(i), pitch_acc(i));
    roll_ikf(i) = pos(1);
    pitch_ikf(i) = pos(3);
     
    
 end
 
 
 
%% plot

%  figure
%  hold on
%  plot(t*samples, vel_ang(:,2), '--')
%  plot(t*samples, gyr_data(:,2))
%  xlabel('t(s)')
%  ylabel('pitch')
%  title('velocidade angular')
%  legend('truth', 'gyr');
%  hold off
 
figure
hold on
plot(rotvec(:,2))
plot(pitch_gyr)
plot(pitch_acc)
plot(pitch_com)
plot(pitch_kf)
plot(pitch_ikf)
xlabel('t(s)')
ylabel('pitch')
title('posição angular')
legend('truth', 'gyr','acc', 'comp.', 'kf','IKF');
hold off

figure
hold on
plot(rotvec(:,1))
plot(roll_gyr)
plot(roll_acc)
plot(roll_com)
plot(roll_kf)
plot(roll_ikf)
xlabel('t(s)')
ylabel('roll')
title('posição angular')
legend('truth', 'roll','acc', 'comp', 'kf', 'IKF');
hold off

%% Calculo do erro
erro_comp_roll = sum((rotvec(:,1) - roll_com).^2)/samples;
erro_kf_roll = sum((rotvec(:,1) - roll_kf).^2)/samples;
erro_ikf_roll = sum((rotvec(:,1) - roll_ikf).^2)/samples;

erro_comp_pitch = sum((rotvec(:,2) - pitch_com).^2)/samples;
erro_kf_pitch = sum((rotvec(:,2) - pitch_kf).^2)/samples;
erro_ikf_pitch = sum((rotvec(:,2) - pitch_ikf).^2)/samples;
disp('                Roll         Pitch')               
disp(['Erro Comp:   ',num2str(erro_comp_roll),'     ', num2str(erro_comp_pitch)])  
disp(['Erro kf:     ',num2str(erro_kf_roll),  '     ', num2str(erro_kf_pitch)]) 
disp(['Erro ikf:    ',num2str(erro_ikf_roll), '     ', num2str(erro_ikf_pitch)]) 
 
 
 
 
