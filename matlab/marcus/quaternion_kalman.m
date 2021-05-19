clear;close all;clc
% leitura do dataset

%verificar os dados gerados pelo script
%../../datasets/simulation/gera_dados.m

% addpath('../../datasets/simulation') % comentado para não bagunçar a
% salva de arquivos, referencias.. 


% achei melhor puxar direto da pasta
movimento_filename = '../../datasets/simulation/movimento.csv';
parado_filename = '../../datasets/simulation/parado.csv';



data = csvread(movimento_filename);
calib_data = csvread(parado_filename);

acc = [data(:,1) data(:,2) data(:,3)];
gyr = [data(:,4) data(:,5) data(:,6)];

acc_calib = [calib_data(:,1) calib_data(:,2) calib_data(:,3)];
gyr_calib = [calib_data(:,4) calib_data(:,5) calib_data(:,6)];

gyr_calibrado = gyr;% - gyr_calib; % não calibra pra vermos o efeito da fusão
acc_calibrado = acc;


%% Modelo
freq = 100;
Ts = 1/freq;
g = [0 0 9.5]'; % gravidade
samples = length(data);

%% Kalman

% Parametros
Qn = 1*diag([1 var(gyr_calib)]);
Qn(4,4) = 0.001; % variancia no eixo z menor para "confiarmos" mais na medida do gyro

Rn = 10*diag([1 var(acc_calib)]);

Pk = 0.1*eye(4); % erro inicial é proximo de zero
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

for i = 1 : samples
    w_measure = [0 gyr_calibrado(i,1) gyr_calibrado(i,2) gyr_calibrado(i,3)]'; %quaterniana
    a_measure = [0 acc_calibrado(i,1) acc_calibrado(i,2) acc_calibrado(i,3)]';
    
    
    
    % predict
    x = (x' + (Ts/2)*quatmultiply(x', w_measure'))';%Assume calibrado (bias constante) %-Ts/2 *quatmultiply(q_k,[0 bias]);
    
    x_pure_gyro = (x_pure_gyro' + (Ts/2)*quatmultiply(x_pure_gyro', w_measure'))';% integração
    
    Jf = (Ts/2)*[2/Ts -w_measure(1) -w_measure(2) -w_measure(3);
        w_measure(1) 2/Ts w_measure(3) -w_measure(2);
        w_measure(2) -w_measure(3) 2/Ts w_measure(1)
        w_measure(3) w_measure(2) -w_measure(1) 2/Ts];
    
    Pk = Jf*Pk*Jf' + Qn;
    
    
    % Update
    a_est = quatrotate(x',g')'; %nav2body
    
    y_est = [0; a_est];
    y_in = a_measure;
    
    Jh =2*g(3)*[0 0 0 0 ;
        -x(3) x(4) -x(1) x(2);
        x(2) x(1) x(4) x(3);
        x(1) -x(2) -x(3) x(4)]; 
    
    Kk = Pk*Jh'*inv(Jh*Pk*Jh'+Rn);
    
    x = x + Kk*(y_in-y_est);  
    
    Pk = (eye(4) - Kk*Jh)*Pk;
    
    % acumuladores
    X(i,:) = x; 
    X_GYRO(i,:) = x_pure_gyro;
end



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

end


