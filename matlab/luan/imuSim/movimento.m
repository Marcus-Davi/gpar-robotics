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


gyr_calibrado = gyr;% - mean(gyr_calib); % não calibra pra vermos o efeito da fusão
acc_calibrado = acc;% - mean(acc_calib);

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
tformMOV = rosmessage('geometry_msgs/TransformStamped');
tformMOV.ChildFrameId = 'EKF';
tformMOV.Header.FrameId = 'map';
tformMOV.Transform.Translation.X = 0;
tformMOV.Transform.Translation.Y = 0;
tformMOV.Transform.Translation.Z = 0;
tformMOV.Transform.Rotation.W = 1;
tformMOV.Transform.Rotation.X = 0;
tformMOV.Transform.Rotation.Y = 0;
tformMOV.Transform.Rotation.Z = 0;

tformESKF = rosmessage('geometry_msgs/TransformStamped');
tformESKF.ChildFrameId = 'ESKF';
tformESKF.Header.FrameId = 'map';
tformESKF.Transform.Translation.X = 0;
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
  
  x = zeros(6,samples(1));
 eskf= quaternionESKF(dt,[0.001 0.001 0.001], vec_gyr_var, vec_acc_var);
 
 for i=1:samples
     %estimação da posição
     eskf.predict(gyr_calibrado(i,:)');
     q_eskf = eskf.update(acc_calibrado(i,:)');
     q_eskf_norm = q_eskf/norm(q_eskf);
     
     %estimação da acc linear
     acc_calibrado_global = quatmultiply(q_eskf_norm', [0, acc_calibrado(i,:)])';
     acc_sgrav_global = acc_calibrado_global - [0;0;g];
     
     %dupla integração
     x(1,i) = x(1,i-1) + x(2,i-1)*dt + acc_sgrav_global(2)*dt*dt/2;
     x(2,i) = x(2,i-1) + acc_sgrav_global(2)*dt;
     x(3,i) = x(3,i-1) + x(4,i-1)*dt + acc_sgrav_global(3)*dt*dt/2;
     x(4,i) = x(4,i-1) + acc_sgrav_global(3)*dt;
     x(5,i) = x(5,i-1) + x(6,i-1)*dt + acc_sgrav_global(4)*dt*dt/2;
     x(6,i) = x(6,i-1) + acc_sgrav_global(4)*dt;
     
    tformMOV.Transform.Translation.X = x(1,i);
    tformMOV.Transform.Translation.Y = x(3,i);
    tformMOV.Transform.Translation.Z = x(5,i);
    tformMOV.Transform.Rotation.W = q_eskf_norm(1);
    tformMOV.Transform.Rotation.X = q_eskf_norm(2);
    tformMOV.Transform.Rotation.Y = q_eskf_norm(3);
    tformMOV.Transform.Rotation.Z = q_eskf_norm(4);
    tformMOV.Header.Stamp = rostime('now');
    sendTransform(tftree,tformMOV);
    
    tformESKF.Transform.Rotation.W = q_eskf_norm(1);
    tformESKF.Transform.Rotation.X = q_eskf_norm(2);
    tformESKF.Transform.Rotation.Y = q_eskf_norm(3);
    tformESKF.Transform.Rotation.Z = q_eskf_norm(4);
    tformESKF.Header.Stamp = rostime('now');
    sendTransform(tftree,tformESKF);
    
    
    pause(dt)

 end
 %%