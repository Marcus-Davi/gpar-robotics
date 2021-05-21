clc, clear, close all
addpath('./filtros', './movimentos')

movimento_filename = '../../../datasets/simulation/movimento.csv';
parado_filename = '../../../datasets/simulation/parado.csv';

%% dados
data = csvread(movimento_filename);
calib_data = csvread(parado_filename);

acc = [data(:,1) data(:,2) data(:,3)];
gyr = [data(:,4) data(:,5) data(:,6)];

acc_calib = [calib_data(:,1) calib_data(:,2) calib_data(:,3)];
gyr_calib = [calib_data(:,4) calib_data(:,5) calib_data(:,6)];

gyr_calibrado = gyr;% - gyr_calib; % não calibra pra vermos o efeito da fusão
acc_calibrado = acc;% - acc_calib;

%% modelo
f = 100; %Hz
dt = 1/f;
g = 9.8056;
samples = size(acc(:,1));
  %% ROS
 rosshutdown
 rosinit
 nome = strcat('pitch_imu','.csv');

    
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

  vec_gyr_var = 1 * [ 1 var(gyr_calib)];
  vec_acc_var  = 10* [ 1 var(acc_calib)];
  vec_gyr_var(4) = 0.001;


 ekf = quaternionEKF(dt, vec_gyr_var, vec_acc_var);
 for i=1:samples
     ekf.predict(gyr_calibrado(i,:)');

     q = ekf.update(acc_calibrado(i,:)');

     q_norm = q/norm(q);

    tform.Transform.Rotation.W = q_norm(1);
    tform.Transform.Rotation.X = q_norm(2);
    tform.Transform.Rotation.Y = q_norm(3);
    tform.Transform.Rotation.Z = q_norm(4);
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
    pause(dt)

 end
 %%
          