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
function [k_roll,k_pitch] = kalman_imu();
%% SETUP
clc;
    display('1.Movimento 2.Calibration');
    [accx accy accz gyrox gyroy gyroz] = arq_imu(0);
    [accx_0 accy_0 accz_0 gyrox_0 gyroy_0 gyroz_0] = imu_calibration();

f = 100; %Hz
T = 1/f;
tam = length(accx);

pitch = zeros(tam,1);
roll = zeros(tam,1);

%% BIAS

accx = accx + accx_0;
accy = accy + accy_0;
accz = accz + accz_0;

gyrox = gyrox + gyrox_0;
gyroy = gyroy + gyroy_0;
gyroz = gyroz + gyroz_0;

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
A = [1 -T 0 0;0 1 0 0; 0 0 1 -T;0 0 0 1];
B = [T 0;0 0;0 T;0 0];

var_q = [0.0035 0;0 0.0037];
Q = B*var_q*B';

H = [1 0 0 0;0 0 1 0];
R = [0.0075 0;0 0.0071];

%% VARIABLES
x = [0;0;0;0];
u = [0;0];
P = eye(4); %Inicialização da matriz de covariância.
x_m = [0;0;0;0];
z = [0;0];

k_pitch = zeros(tam,1);
k_roll = zeros(tam,1);

% x --> previous state vector
% x_ --> predicted state vector
% P --> previous covariance matrix
% P_ --> predicted covariance matrix
%% KALMAN FILTER
for i=1:tam
%Prediction
    u = [gyrox(i);gyroy(i)]; 
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

end