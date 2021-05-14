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
 
 gyr_params = gyroparams(...Senso
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
 
  %% ROS
 rosshutdown
 rosinit
 nome = strcat('pitch_imu','.csv');
 
while(1)

    angle_x = 0;
    angle_y = 0;
    angle_z = 0;

    % Ros data
    tftree = rostf;
    tform = rosmessage('geometry_msgs/TransformStamped');
    tform.ChildFrameId = 'imu';
    tform.Header.FrameId = 'map';
    tform.Transform.Translation.X = 0;
    tform.Transform.Translation.Y = 0;
    tform.Transform.Translation.Z = 0;
    tform.Transform.Rotation.W = 1;
    tform.Transform.Rotation.X = 0;
    tform.Transform.Rotation.Y = 0;
    tform.Transform.Rotation.Z = 0;

     %% Filtro de Kalman
     
     vec_gyr_bias = [ 0 gyr_bias gyr_bias gyr_bias];
     vec_gyr_var  = [ 0 gyr_var  gyr_var  gyr_var];
     
     ekf = quaternionEKF(dt, vec_gyr_bias, vec_gyr_var);
     for i=1:samples
         ekf.predict(gyr_data(i,:));
         
         q = ekf.update(acc_data(i,:)');
         
        tform.Transform.Rotation.W = q(1);
        tform.Transform.Rotation.X = q(2);
        tform.Transform.Rotation.Y = q(3);
        tform.Transform.Rotation.Z = q(4);
        tform.Header.Stamp = rostime('now');
        sendTransform(tftree,tform);
        pause(dt)
        
     end
end
          