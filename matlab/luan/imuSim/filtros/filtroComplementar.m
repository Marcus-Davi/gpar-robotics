function [comp] = filtroComplementar(accAngle,gyrAddAngle, previousAngle)
%FILTROCOMPLEMENTAR Summary of this function goes here
%   Detailed explanation goes here

    comp = 0.98 * (previousAngle+gyrAddAngle) + (1-0.98) * accAngle;
end

