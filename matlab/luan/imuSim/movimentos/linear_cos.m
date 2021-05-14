function [acc_linear,vel_angular, rotvec] = linear_cos(f, samples, w)
%   Funcão vai retornar um movimento linear em x
% respeitando a seguinte regra
%
%  x = 1 * cos(w*t)
%
%  Luan Amaral

t = (0:(1/f):(samples -1)/f)';

x = -w*w * cos(w*t); 

acc_linear  = [ zeros(1,samples)' , x                , zeros(1,samples)'];
vel_angular = zeros(samples,3);
rotvec = zeros(samples,3);
end

