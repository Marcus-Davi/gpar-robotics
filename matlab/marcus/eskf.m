 %clear;close all;clc
% leitura do dataset

%verificar os dados gerados pelo script
%../../datasets/simulation/gera_dados.m

% addpath('../../datasets/simulation') % comentado para não bagunçar a
% salva de arquivos, referencias..


% achei melhor puxar direto da pasta
movimento_filename = '../../datasets/simulation/movimento.csv';
parado_filename = '../../datasets/simulation/parado.csv';
ground_truth_filename = '../../datasets/simulation/ground_truth.csv';



data = csvread(movimento_filename);
calib_data = csvread(parado_filename);
ground_truth = csvread(ground_truth_filename);

acc = [data(:,1) data(:,2) data(:,3)];
gyr = [data(:,4) data(:,5) data(:,6)];

%dados obtidos com sensor inerte
acc_calib = [calib_data(:,1) calib_data(:,2) calib_data(:,3)];
gyr_calib = [calib_data(:,4) calib_data(:,5) calib_data(:,6)];

acc_caluib_mean = mean(acc_calib);
gyr_calib_mean = mean(gyr_calib); %bias

% gyr_calib_mean(3) = -0.05; % bias artificia
% gyr_calib_mean(2) = -0.05; % bias artificial
% gyr_calib_mean(1) = -0.05; % bias artificial

gyr_calibrado = gyr- gyr_calib_mean; %remove bias
acc_calibrado = acc;


%% Modelo
freq = 400; % precisa ser o mesmo do gera_dados.m
Ts = 1/freq;
g = [0 0 9.81]'; % gravidade "errada" pra reduzir instabilidade
samples = length(data);

%% Kalman1
% Parametros Kalman
Qn_gyr = Ts^2*diag(var(gyr_calib));
Qn_bias = Ts*diag([0.001 0.001 0.001]);
Qn = blkdiag(Qn_gyr,Qn_bias);
% Qn(4,4) = 0.00001; % variancia no eixo z menor para "confiarmos" mais na medida do gyro

Rn = 1*diag(var(acc_calib));
% Rn(4,4) = 1;

Pk = zeros(6);

x = [1 0 0 0]'; %fusion
x_pure_gyro = [1 0 0 0]'; %gyro only
%% ROS
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





%% Loop

X = zeros(samples,4);
X_GYRO = zeros(samples,4);

w_bias = [0 0 0]';
for i = 1 : samples
    w_measure = [gyr_calibrado(i,1) gyr_calibrado(i,2) gyr_calibrado(i,3)]'; %quaterniana
    a_measure = [acc_calibrado(i,1) acc_calibrado(i,2) acc_calibrado(i,3)]';
    
    % predict
    uk = [0; w_measure - w_bias]';
    x = (x' + (Ts/2)*quatmultiply(x', uk))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]);
    w_bias = w_bias;
    
    
 
    Jf = [quat2rotm(uk*Ts) -Ts*eye(3);
        zeros(3) eye(3)];
    Pk = Jf*Pk*Jf' + Qn;
    
    if i > 1
        %jacobiana da medida em relação ao erro
        % H = d h(x)/ dx * dx / d(deltax)
        Hx = 2*g(3)*[-x(3) x(4) -x(1) x(2);
            x(2) x(1) x(4) x(3);
            x(1) -x(2) -x(3) x(4)];
        
        Hdx = 1/2 * [-x(2) -x(3) -x(4);
            x(1) -x(4) x(3);
            x(4) x(1) -x(2);
            -x(3) x(2) x(1)];
        
        H = [Hx*Hdx zeros(3)];
        
        Kk = Pk*H'*inv(H*Pk*H' + Rn);
        
        a_est = quatrotate(x',g')'; %nav2body
        
        dX = Kk*(a_measure - a_est);

        deltaRot = dX(1:3);
        deltaRotNorm = norm(deltaRot);
        deltaRotQuat = [cos(deltaRotNorm/2);
            (deltaRot/deltaRotNorm)*sin(deltaRotNorm/2)];
        
        
        
        x = quatmultiply(x',deltaRotQuat')';
        w_bias = w_bias + dX(4:end);
        
        Pk = (eye(6) - Kk*H)*Pk;
        
    end
    
    
    % acumuladores
    X(i,:) = x;
    X_GYRO(i,:) = x_pure_gyro;
end
euler = quat2eul(X,'XYZ');
euler_true = quat2eul(ground_truth,'XYZ');

subplot(3,1,1)
plot(euler(:,1),'--')
hold on
plot(euler_true(:,1))
legend('roll','true roll')

subplot(3,1,2)
plot(euler(:,2),'--')
hold on
plot(euler_true(:,2))
legend('pitch','true pitch')

subplot(3,1,3)
plot(euler(:,3),'--')
hold on
plot(euler_true(:,3))
legend('yaw','true yaw')


return
%% Visualization
for i = 1 : samples
    x = X(i,:);
    x_pure_gyro = X_GYRO(i,:);
    
    x = quatnormalize(x);
    x_pure_gyro = quatnormalize(x_pure_gyro);
    
    
    tform.Transform.Rotation.W = x(1);
    tform.Transform.Rotation.X = x(2);
    tform.Transform.Rotation.Y = x(3);
    tform.Transform.Rotation.Z = x(4);
    tform.Header.Stamp = rostime('now');
    sendTransform(tftree,tform);
    
    % gyro only tf
    tform2.Transform.Rotation.W = x_pure_gyro(1);
    tform2.Transform.Rotation.X = x_pure_gyro(2);
    tform2.Transform.Rotation.Y = x_pure_gyro(3);
    tform2.Transform.Rotation.Z = x_pure_gyro(4);
    tform2.Header.Stamp = rostime('now');
    sendTransform(tftree,tform2);
    
    pause(Ts)
    i
    
end


