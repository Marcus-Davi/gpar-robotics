%{
Esse programa vai trabalhar só com a questão da velocidade angular, fazer
outro para caso eu deseje trabalhar com a linear.
%}

% Usado na classe IMU do matlab. Ainda relevante para o GPAR, mas vou
% juntar com o outro programa
function [orientation angVel acc_linear roll pitch yaw] = motion_ang(N);

%% Definição de variáveis
Fs = 100; % frequência Hz

%N --> amostras

t = (0:(1/Fs):(N - 1)/Fs)';
%{
Explicação sobre a estrutura do tempo.
Começamos do tempo 0s
O tempo final será a quantidade de amostras - 1 * o tempo por amostra, esse
tempo é 1/Fs. E logo esse 1/Fs também será o incremento do meu vetor tempo.
E por fim, ele está transposto para termos um vetor (N,1)
%}

roll = zeros(N,1);
pitch = zeros(N,1);
yaw = zeros(N,1);

vel_roll = zeros(N,1);
vel_pitch = zeros(N,1);
vel_yaw = zeros(N,1);

% Acelerações Lineares
accx = zeros(N,1);
accy = zeros(N,1);
accz = zeros(N,1);

w = 2; % frequência angular (rad/s), usada em sinais senoidais
%% Definindo sinais usados
for i=1:N
    if(i <= N/2)
    pitch(i) = 0;
    vel_pitch(i) = 0 ;
    yaw(i) = 0.5*cos((2*pi/N))*i);
    vel_yaw(i) = -(pi/N)*sin((2*pi/N)*i);
    else
    roll(i) = 0;
    vel_roll(i) = 0;
    end  
end
%% Retorno

angVel = [vel_roll vel_pitch vel_yaw];

rotvec = [roll pitch yaw];
orientation = quaternion(rotvec,'rotvec');

acc_linear = [accx accy accz];
end