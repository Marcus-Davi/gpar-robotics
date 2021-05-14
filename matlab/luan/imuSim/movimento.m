%-------------------------------------------------------------------------%
% Programa para detectar movimento com base na leitura de um IMU          %                  
%                                                                         %
%     Luan Amaral                                                         % 
%                                                                         %
%-------------------------------------------------------------------------%
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

%% Imu inicialização
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

 %% Leitura

 samples = 1000;
 t = (0:(1/f):(samples -1)/f);
 
%  [acc_lin, vel_ang, rotvec] = linear_cos(f, samples, 1);
[acc_lin, vel_ang, rotvec] = parado(samples);
 orientation = quaternion(rotvec, 'rotvec');
 
 [acc_data, gyr_data] = imu(acc_lin, vel_ang, orientation);
 
 %% detecção do movimento

 %filtro passa baixa
 for i=2:samples
    acc_data(i,:) = 0.1*acc_data(i,:) + 0.9*acc_data(i-1,:);  
 end
 
 modulo = zeros(samples, 1);
 moveu = zeros(samples, 1);
 
 tol = sqrt(3*(acc_var*acc_var));
 
 for i=2:samples
     modulo(i) = sqrt(sum(acc_data(i,:).^2));
     if( moveu(i-1) == 1 )
         ntol = tol/3;
     else
         ntol = tol;
     end       
    if (abs(modulo(i)-g) < ntol)
        %não houve aceleração
    else
        moveu(i) = 1;
    end    
 end
 
 %% Plots
 
 figure
 hold on
 plot(moveu)
 title('MOVEU')
 hold off
 
 
