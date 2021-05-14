function [acc_linear, vel_angular, rotvec] = angular_cos(f, samples, w)
%   Funcão vai retornar um movimento angular em pitch 
% respeitando a seguinte regra
%
%  pitch = pi/2 * cos(w*t)
%
%  Luan Amaral

t = (0:(1/f):(samples -1)/f)';

pitch = pi/2 * cos(w*t);
w_pitch = -pi*w/2 * sin(w*t); 

rotvec = [zeros(samples,1), pitch, zeros(samples,1)];

acc_linear  = [ zeros(samples,1), zeros(samples,1), zeros(samples,1) ];
vel_angular = [ zeros(samples,1), w_pitch         , zeros(samples,1) ];
end

