clc, clear, close all
addpath('./filtros', './movimentos')

movimento_filename = '../../../datasets/rosbags/street_imu.csv';
% movimento_filename = '../../../datasets/simulation/movimento.csv';
parado_filename = '../../../datasets/simulation/parado.csv';
ground_truth_filename = '../../../datasets/simulation/ground_truth.csv';

%% dados
data = csvread(movimento_filename);
calib_data = csvread(parado_filename);
ground_truth = csvread(ground_truth_filename);

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

% tform = rosmessage('geometry_msgs/TransformStamped');
% tform.ChildFrameId = 'true';
% tform.Header.FrameId = 'map';
% tform.Transform.Translation.X = 0;
% tform.Transform.Translation.Y = 0;
% tform.Transform.Translation.Z = 0;
% tform.Transform.Rotation.W = 1;
% tform.Transform.Rotation.X = 0;
% tform.Transform.Rotation.Y = 0;
% tform.Transform.Rotation.Z = 0;

tform2 = rosmessage('geometry_msgs/TransformStamped');
tform2.ChildFrameId = 'imu';
tform2.Header.FrameId = 'map';
tform2.Transform.Translation.X = 0;
tform2.Transform.Translation.Y = 0;
tform2.Transform.Translation.Z = 0;
tform2.Transform.Rotation.W = 1;
tform2.Transform.Rotation.X = 0;
tform2.Transform.Rotation.Y = 0;
tform2.Transform.Rotation.Z = 0;

 %% Filtro de Kalman

  vec_gyr_var = 1 * [var(gyr_calib)];
  vec_acc_var  = 10* [ 1 var(acc_calib)];
  vec_gyr_var(3) = 0.001;
  
  x = zeros(6,samples(1));
  
 quat_eskf = zeros(4,samples(1));
 eskf= quaternionESKF(dt,[0.001 0.001 0.001], vec_gyr_var, 15*vec_acc_var);

 for i=1:samples
     %estimação da posição
     eskf.predict(gyr_calibrado(i,:)');
     if i>1
        q_eskf = eskf.update(acc_calibrado(i,:)');
        quat_eskf(:,i) = q_eskf;
        q_eskf_norm = q_eskf/norm(q_eskf);   
        
%         tform.Transform.Rotation.W = ground_truth(i,1);
%         tform.Transform.Rotation.X = ground_truth(i,2);
%         tform.Transform.Rotation.Y = ground_truth(i,3);
%         tform.Transform.Rotation.Z = ground_truth(i,4);
%         tform.Header.Stamp = rostime('now');
%         sendTransform(tftree,tform);

        tform2.Transform.Rotation.W = q_eskf_norm(1);
        tform2.Transform.Rotation.X = q_eskf_norm(2);
        tform2.Transform.Rotation.Y = q_eskf_norm(3);
        tform2.Transform.Rotation.Z = q_eskf_norm(4);
        tform2.Header.Stamp = rostime('now');
        sendTransform(tftree,tform2);
    end
    
    pause(dt)

 end
 
 %% plot
 euler = quat2eul(quat_eskf', 'XYZ');
 euler_true = quat2eul(ground_truth, 'XYZ');
 
subplot(3,1,1)
plot(euler(:,1),'--')
hold on
% plot(euler_true(:,1))
legend('roll','true roll')

subplot(3,1,2)
plot(euler(:,2),'--')
hold on
% plot(euler_true(:,2))
legend('pitch','true pitch')

subplot(3,1,3)
plot(euler(:,3),'--')
hold on
% plot(euler_true(:,3))
legend('yaw','true yaw')

 
 
 