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
function [velo_truth,posi_truth,accx ,accy , accz , gyrox , gyroy , gyroz] = imu(N,i);
% Para i = 1 trabalhamos com algum movimento, para diferente de 1
% consideramos movimento nulo

gyro_p = gyroparams( ...
            'MeasurementRange',4.3633, ...
            'Resolution',1.3323e-04, ...
            'ConstantBias',0.03491, ...
            'AxesMisalignment',2, ...
            'NoiseDensity',8.7266e-4, ...
            'BiasInstability',0.05, ...
            'TemperatureScaleFactor',0.02,...
            'TemperatureBias',0.349,...
            'AccelerationBias',0.178e-3);

accel_p = accelparams( ...
            'MeasurementRange',19.62, ...
            'Resolution',0.00059875, ...
            'ConstantBias',0.09, ...
            'AxesMisalignment',2, ...
            'NoiseDensity',3.92e-3, ...
            'TemperatureBias',[0.34335 0.34335 0.5886], ...
            'TemperatureScaleFactor',0.02, ...
             'BiasInstability', 0.05);
        
IMU = imuSensor('accel-gyro');    
IMU.Gyroscope = gyro_p;
IMU.Accelerometer = accel_p;

if(i == 1)
[orientation angVel acc_linear roll pitch yaw] = motion_ang(N);

velo_truth = zeros(N,1);
posi_truth = zeros(N,1);

dt = 1/100;

for i = 2:N
  velo_truth(i) = velo_truth(i-1) + (acc_linear(i,1)+acc_linear(i-1,1))*dt/2;  
  posi_truth(i) = posi_truth(i-1) + (velo_truth(i)+velo_truth(i-1))*dt/2;  
end

[accelReadings , gyroReadings] = IMU(acc_linear,angVel,orientation);

else
    velo_truth = zeros(N,1);
    posi_truth = zeros(N,1);
    angVel = zeros(N,3);
    acc_linear = zeros(N,3);
    
    [accelReadings , gyroReadings] = IMU(acc_linear,angVel);
end

accx = accelReadings(:,1);
accy = accelReadings(:,2);
accz = accelReadings(:,3);

gyrox = gyroReadings(:,1);
gyroy = gyroReadings(:,2);
gyroz = gyroReadings(:,3);

end

            

