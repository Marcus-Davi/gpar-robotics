%IMU calibration

function [accx_0 accy_0 accz_0 gyrox_0 gyroy_0 gyroz_0] = imu_calibration();
%% SETUP
[accx accy accz gyrox gyroy gyroz] = arq_imu(0);

N = length(accx);     
%% BIAS CALCULATION

bias_ax = sum(accx(1:N))/N;
bias_ay = sum(accy(1:N))/N;
bias_az = sum(accz(1:N))/N;

bias_gx = sum(gyrox(1:N))/N;
bias_gy = sum(gyroy(1:N))/N;
bias_gz = sum(gyroz(1:N))/N;

accx_0 = 0 - bias_ax;
accy_0 = 0 - bias_ay;
accz_0 = 9.8065 - bias_az;

gyrox_0 = 0 - bias_gx;
gyroy_0 = 0 - bias_gy;
gyroz_0 = 0 - bias_gz;
end