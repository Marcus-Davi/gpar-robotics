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
f = 400; %Hz
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
tformEKF = rosmessage('geometry_msgs/TransformStamped');
tformEKF.ChildFrameId = 'EKF';
tformEKF.Header.FrameId = 'map';
tformEKF.Transform.Translation.X = 0;
tformEKF.Transform.Translation.Y = 0;
tformEKF.Transform.Translation.Z = 0;
tformEKF.Transform.Rotation.W = 1;
tformEKF.Transform.Rotation.X = 0;
tformEKF.Transform.Rotation.Y = 0;
tformEKF.Transform.Rotation.Z = 0;

tformESKF = rosmessage('geometry_msgs/TransformStamped');
tformESKF.ChildFrameId = 'ESKF';
tformESKF.Header.FrameId = 'map';
tformESKF.Transform.Translation.X = 0.5;
tformESKF.Transform.Translation.Y = 0;
tformESKF.Transform.Translation.Z = 0;
tformESKF.Transform.Rotation.W = 1;
tformESKF.Transform.Rotation.X = 0;
tformESKF.Transform.Rotation.Y = 0;
tformESKF.Transform.Rotation.Z = 0;


 %% Filtro de Kalman

  vec_gyr_var = 1 * [ 1 var(gyr_calib)];
  vec_acc_var  = 10* [ 1 var(acc_calib)];
  vec_gyr_var(4) = 0.001;


 ekf = quaternionEKF(dt, vec_gyr_var, vec_acc_var);
 eskf= quaternionESKF(dt, vec_gyr_var, vec_acc_var);
 
 for i=1:samples
     ekf.predict(gyr_calibrado(i,:)');
     eskf.predict(gyr_calibrado(i,:)');

     q_ekf = ekf.update(acc_calibrado(i,:)');
     q_eskf = ekf.update(acc_calibrado(i,:)');

     q_ekf_norm = q_ekf/norm(q_ekf);
     q_eskf_norm = q_eskf/norm(q_eskf);

    tformEKF.Transform.Rotation.W = q_ekf_norm(1);
    tformEKF.Transform.Rotation.X = q_ekf_norm(2);
    tformEKF.Transform.Rotation.Y = q_ekf_norm(3);
    tformEKF.Transform.Rotation.Z = q_ekf_norm(4);
    tformEKF.Header.Stamp = rostime('now');
    sendTransform(tftree,tformEKF);
    
    tformESKF.Transform.Rotation.W = q_eskf_norm(1);
    tformESKF.Transform.Rotation.X = q_eskf_norm(2);
    tformESKF.Transform.Rotation.Y = q_eskf_norm(3);
    tformESKF.Transform.Rotation.Z = q_eskf_norm(4);
    tformESKF.Header.Stamp = rostime('now');
    sendTransform(tftree,tformESKF);
    
    
    pause(dt)

 end
 %%
          