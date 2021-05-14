% IMU complementary Filter

function [cf_pitch cf_roll] = imu_cf();
%% SETUP
clc;

    [accx accy accz gyrox gyroy gyroz] = arq_imu(0);
    [accx_0 accy_0 accz_0 gyrox_0 gyroy_0 gyroz_0] = imu_calibration();

f = 100; %Hz   
tam = length(accx);
%% BIAS

accx = accx + accx_0;
accy = accy + accy_0;
accz = accz + accz_0;

gyrox = gyrox + gyrox_0;
gyroy = gyroy + gyroy_0;
gyroz = gyroz + gyroz_0;

%% ACCELEROMETER ANGLE
acc_p = zeros(tam,1);
acc_r = zeros(tam,1);

for i=1:tam
   acc_p(i) = -atan2(accx(i),sqrt(accz(i)*accz(i)+accy(i)*accy(i)));
   acc_r(i) = atan2(accy(i),sqrt(accz(i)*accz(i)+accx(i)*accx(i)));
end
%% GYROSCOPE ANGLE
gyro_x = zeros(tam,1);
gyro_y = zeros(tam,1);

for i=2:tam
   gyro_y(i) = gyro_y(i-1)+gyroy(i-1)*1/f;
   gyro_x(i) = gyro_x(i-1)+gyrox(i-1)*1/f;
end
%% COMPLEMENTARY FILTER
cf_pitch = zeros(tam,1);
cf_roll = zeros(tam,1);
a = 0.98;
for i=2:tam
   cf_pitch(i) = a*(cf_pitch(i-1) + gyro_y(i)-gyro_y(i-1)) + (1-a)*(acc_p(i));
   cf_roll(i) = a*(cf_roll(i-1) + gyro_x(i)-gyro_x(i-1)) + (1-a)*(acc_r(i));
end

%% PLOTAGEM
%{
[orientation angVel roll pitch yaw] = motion_ang(tam);

subplot(1,2,1);
plot(cf_roll,'-b');
hold on;
plot(roll,'-g');
legend('cf roll','truth roll');
ylim([-1.7 1.7]);
title('ROLL');

subplot(1,2,2);
plot(cf_pitch,'-b');
hold on;
plot(pitch,'-g');
legend('cf pitch','truth pitch');
ylim([-1.7 1.7]);
title('PITCH');
%}
end