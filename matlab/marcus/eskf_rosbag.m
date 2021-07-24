close all;clc
% leitura do dataset

%verificar os dados gerados pelo script
%../../datasets/simulation/gera_dados.m

% addpath('../../datasets/simulation') % comentado para não bagunçar a
% salva de arquivos, referencias..


% achei melhor puxar direto da pasta
movimento_filename = '../../datasets/rosbags/street_imu.csv';
data = csvread(movimento_filename);
% calib_data = csvread(parado_filename);
% ground_truth = csvread(ground_truth_filename);

acc = [data(:,1) data(:,2) data(:,3)];
gyr = [data(:,4) data(:,5) data(:,6)];

%dados obtidos com sensor inerte
% acc_calib = [calib_data(:,1) calib_data(:,2) calib_data(:,3)];
% gyr_calib = [calib_data(:,4) calib_data(:,5) calib_data(:,6)];
%
% acc_caluib_mean = mean(acc_calib);
% gyr_calib_mean = mean(gyr_calib); %bias

% gyr_calib_mean(3) = -0.05; % bias artificia
% gyr_calib_mean(2) = -0.05; % bias artificial
% gyr_calib_mean(1) = -0.05; % bias artificial


gyr_calibrado = gyr;
acc_calibrado = acc;


%% Modelo
freq = 100; % precisa ser o mesmo do gera_dados.m
Ts = 1/freq;
samples = length(data);

%% Kalman1
% Parametros Kalman
Qn_gyr = Ts^2*diag([0.005 0.005 0.005]); %output noise density = 0.3 º/s (rms) @ 47 Hz
Qn_acc_bias = Ts*diag([1e-8 1e-8 1e-8]); % ?
Qn_gyr_bias = Ts*diag([1e-8 1e-8 1e-8]); % ?
Qn_grav = zeros(3);
Qn = blkdiag(Qn_gyr,Qn_acc_bias,Qn_gyr_bias,Qn_grav);

% Qn(4,4) = 0.00001; % variancia no eixo z menor para "confiarmos" mais na medida do gyro

Rn = 1*diag([0.1 0.1 0.1]); % acc (measurement)
% Rn(4,4) = 1;

Pk = zeros(12);

x_pure_gyro = [1 0 0 0]'; %gyro only

%% Loop
%stados -> quaternion, a_bias , w_bias, gravity
X = zeros(samples,4);
X_GYRO = zeros(samples,4);

quat = [1 0 0 0]'; %fusion
gravity = [0 0 0.81]';
w_bias = [0 0 0]';
a_bias = [0 0 0]';
for i = 1 : samples
    
    w_measure = [gyr_calibrado(i,1) gyr_calibrado(i,2) gyr_calibrado(i,3)]'; %quaterniana
    a_measure = [acc_calibrado(i,1) acc_calibrado(i,2) acc_calibrado(i,3)]';
    
    % Propagation / Predict
    w_compensated = w_measure - w_bias;
    a_compensated = a_measure - a_bias;
    
    x_delta = getQuatDelta(w_compensated*Ts);
    
    quat = quatmultiply(quat',x_delta')';
    a_bias = a_bias;
    w_bias = w_bias;
    gravity = gravity;
    
    
    Jf = [quat2rotm(x_delta')' zeros(3) -Ts*eye(3) zeros(3);
        zeros(3) eye(3) zeros(3) zeros(3);
        zeros(3) zeros(3) eye(3) zeros(3);
        zeros(3) zeros(3) zeros(3) eye(3)]
    Pk = Jf*Pk*Jf' + Qn;
    
    if i > 1
        %jacobiana da medida em relação ao erro
        % H = d h(x)/ dx * dx / d(deltax)
        Hx = 2*gravity(3)*[-quat(3) quat(4) -quat(1) quat(2);
            quat(2) quat(1) quat(4) quat(3);
            quat(1) -quat(2) -quat(3) quat(4)];
        
        Hx_g = [0 0 quat(2)*quat(4)-quat(1)*quat(3);
            0 0 quat(3)*quat(4)+quat(1)*quat(2);
            0 0 quat(4)*quat(4)-quat(3)*quat(3)-quat(2)*quat(2)+quat(1)*quat(1)];
        
        Hx = [Hx zeros(3) zeros(3) Hx_g];
        
        Hdx = 1/2 * [-quat(2) -quat(3) -quat(4);
            quat(1) -quat(4) quat(3);
            quat(4) quat(1) -quat(2);
            -quat(3) quat(2) quat(1)];
        
        Hdx = blkdiag(Hdx,eye(9));
        
        Hxdx = Hx*Hdx;
        
        Kk = Pk*Hxdx'*inv(Hxdx*Pk*Hxdx' + Rn);
        
        a_est = quatrotate(quat',gravity')'; %nav2body
        
        dX = Kk*(a_compensated - a_est);
        
        % compose error states
        deltaRot = dX(1:3);
        deltaABias = dX(4:6);
        deltaGBias = dX(7:9);
        deltaGrav = dX(10:12);    
        deltaRotQuat = getQuatDelta(deltaRot);
           
        % BOX SUM
        quat = quatmultiply(quat',deltaRotQuat')';
        a_bias = a_bias + deltaABias;
        w_bias = w_bias + deltaGBias;
        gravity = gravity + deltaGrav;
        
        Pk = (eye(12) - Kk*Hxdx)*Pk;
        
    end
    
    
    % acumuladores
    X(i,:) = quat;
    X_GYRO(i,:) = x_pure_gyro;
end
euler = quat2eul(X,'XYZ');

subplot(3,1,1)
plot(euler(:,1),'--')
subplot(3,1,2)
plot(euler(:,2),'--')
subplot(3,1,3)
plot(euler(:,3),'--')



% return
%% Visualization
% ROS
rosshutdown % desligar antes
rosinit % roscore

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


% Gyro
tform2 = rosmessage('geometry_msgs/TransformStamped');
tform2.ChildFrameId = 'gyro';
tform2.Header.FrameId = 'map';
tform2.Transform.Translation.X = 0.5; % só pra ficar distante e melhorar visualização
tform2.Transform.Translation.Y = 0;
tform2.Transform.Translation.Z = 0;
tform2.Transform.Rotation.W = 1;
tform2.Transform.Rotation.X = 0;
tform2.Transform.Rotation.Y = 0;
tform2.Transform.Rotation.Z = 0;



for i = 1 : samples
    quat = X(i,:);
    x_pure_gyro = X_GYRO(i,:);
    
    quat = quatnormalize(quat);
    x_pure_gyro = quatnormalize(x_pure_gyro);
    
    
    tform.Transform.Rotation.W = quat(1);
    tform.Transform.Rotation.X = quat(2);
    tform.Transform.Rotation.Y = quat(3);
    tform.Transform.Rotation.Z = quat(4);
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
    
    % gyro only tf
    tform2.Transform.Rotation.W = x_pure_gyro(1);
    tform2.Transform.Rotation.X = x_pure_gyro(2);
    tform2.Transform.Rotation.Y = x_pure_gyro(3);
    tform2.Transform.Rotation.Z = x_pure_gyro(4);
    tform2.Header.Stamp = rostime('now');
    sendTransform(tftree,tform2);
    
    %     pause(Ts)
    i
    
end


function y = getQuatDelta(angles)
angles_norm = norm(angles);
u = cos(angles_norm/2);
v = (angles / angles_norm) * sin(angles_norm/2);
v = reshape(v,3,1);
y = [u;v];
end

