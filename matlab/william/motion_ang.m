%{
Esse programa vai trabalhar só com a questão da velocidade angular, fazer
outro para caso eu deseje trabalhar com a linear.

%}
function [orientation angVel roll pitch yaw] = motion_ang(N);

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

w = 2; % frequência angular (rad/s), usada em sinais senoidais
%% Definindo sinais usados
for i=1:N
    if(i <= 473)
    pitch(i) = (pi/2)*sin(w*t(i));
    vel_pitch(i) =(pi/2)*w*cos(w*t(i));
    else
    roll(i) = (pi/2)*sin(w*t(i));
    vel_roll(i) = (pi/2)*w*cos(w*t(i));
    end
end
%% Retorno

angVel = [vel_roll vel_pitch vel_yaw];

rotvec = [roll pitch yaw];
orientation = quaternion(rotvec,'rotvec');
end