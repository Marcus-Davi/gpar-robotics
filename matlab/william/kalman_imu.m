%{
-->Kalman Filter do IMU

* Nosso sistema vai calcular o ângulo pitch e roll com auxílio do
giroscópio e acelerômetro. As equações que serão usadas para a predição
são:

w_roll = w_roll_true + w_bias 

roll_k = roll_k-1 + dt*(w_roll_k - w_roll_bias_k-1)

O mesmo vale para o pitch.
Nosso STATE VECTOR será então:

roll
w_roll_bias
pitch
w_pitch_bias

Nossa medição com o acelerômetro será sobre o roll e pitch.
Nossa CONTROL VARIABLE, u, será a velocidade angular do giroscópio, logo

w_roll
w_pitch

--> COVARIANCE
Nossa matriz não tem um formato definido. Ela vai se alterando de acordo
com o processo, diminuindo seu valor com a precisão do nosso sistema. 

P_k = A.P_k-1.A' + Q --> Covariance Extrapolation Equation

Porém temos um formato para Q
Q -->
Q = B.var.B'

Esse VAR indica a variância das nossas variáveis de controle. Como estamos
trabalhando com duas variáveis de controle, W_ROLL e W_PITCH então vamos
calcular a variância delas. 

var = [var_w_roll 0;0 var_w_pitch];
%}

%% SETUP
clc;
motion_filename = './data/movement_roll.csv';
stationary_filename = './data/stationary.csv';

data = csvread(motion_filename);
calib = csvread(stationary_filename);

f = 100; %Hz
dt = 1/f;

% Accelerometer Data
accx = data(:,1);
accy = data(:,2);
accz = data(:,3);

% Gyroscope Data
gyrox = data(:,4);
gyroy = data(:,5);
gyroz = data(:,6);

tam = length(accx);

pitch = zeros(tam,1);
roll = zeros(tam,1);

%% CALIBRATION
calib_acc = [calib(:,1) calib(:,2) calib(:,3)];
calib_gyro = [calib(:,4) calib(:,5) calib(:,6)];

mean_calib_acc = mean(calib_acc);
mean_calib_gyro = mean(calib_gyro);

%%{
accx = accx - mean_calib_acc(1,1);
accy = accy - mean_calib_acc(1,2);
accz = accz - (9.8 - mean_calib_acc(1,3));

gyrox = gyrox - mean_calib_gyro(1,1);
gyroy = gyroy - mean_calib_gyro(1,2);
gyroz = gyroz - mean_calib_gyro(1,3);
%}
%% MEASUREMENT

pitch = -atan2(accx,sqrt(accz.*accz+accy.*accy)); 
roll = atan2(accy,sqrt(accz.*accz+accx.*accx));

%% MATRICES
%{
x = Ax + Bu
P = APA' + Q
Q = B.var.B'
z = Hx_m 
K = PH'(HPH' + R)^-1

x --> 4x1
A --> 4x4
u --> 2x1
B --> 4x2

P --> 4x4
Q --> 4x4
var --> 2x2

z --> 2x1
x_m --> 4x1
H --> 2x4

R --> 2x2
%}
A = [1 -dt 0 0;0 1 0 0; 0 0 1 -dt;0 0 0 1];
B = [dt 0;0 0;0 dt;0 0];

%var_q = [0.0035 0;0 0.0037];
var_gyro = var(calib_gyro);
var_q = [var_gyro(1) 0;0 var_gyro(2)]; 

Q = B*var_q*B';

H = [1 0 0 0;0 0 1 0];
var_acc = var(calib_acc);
var_r = [var_acc(1) 0;0 var_acc(2)];
%R = [0.0075 0;0 0.0071];
R = var_r;

%% VARIABLES
x = [0 0 0 0]';
u = [0 0]';
P = eye(4); %Inicialização da matriz de covariância.
x_m = [0 0 0 0]';
z = [0 0]';

k_pitch = zeros(tam,1);
k_roll = zeros(tam,1);

% x --> previous state vector
% x_ --> predicted state vector
% P --> previous covariance matrix
% P_ --> predicted covariance matrix
%% KALMAN FILTER
for i=1:tam
%Prediction
    u = [gyrox(i) gyroy(i)]'; 
    x_ = A*x + B*u;
    P_ = A*P*A' + Q;
% Measurement
    x_m = [roll(i);0;pitch(i);0];
    z = H*x_m;
% Update
    K = P_*H'*(H*P_*H' + R)^-1;
    x = x_ + K*(z - H*x_);
    P = (eye(4)-K*H)*P_*(eye(4)-K*H)' + K*R*K';
% Output
    k_pitch(i) = x(3,1);
    k_roll(i) = x(1,1);
end

%% PLOTING
    subplot(2,1,1);
    plot(k_roll,'--');
    title('ROLL');
    
    subplot(2,1,2);
    plot(k_pitch,'--');
    title('PITCH');