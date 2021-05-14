function [acc_linear, vel_angular, rotvec] = escada_linear_y(f, samples, l)
%   Funcão vai retornar um movimento linear em y 
% respeitando a seguinte regra
%
%  vel(y) = l/t se t pertence a [0.2t,0.4t], [0.6t, 0.8t]  
%         = 0   se t pertence a [0, 0.2t), (0.4t,0.6t)
%
%  Luan Amaral

tt = (samples -1)/f;
t = (0:(1/f):tt)';
dt = t(2)-t(1);
acc_linear = zeros(samples, 3);
vel_linear = zeros(samples, 3);
vel_angular= zeros(samples, 3);
rotvec = zeros(samples, 3);

for i=samples*.2:samples*.4
    vel_linear(i) =  l/tt;
    acc_linear(i) = (vel_linear(i)-vel_linear(i-1))/dt;
end
end

