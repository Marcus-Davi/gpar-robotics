%Código do complementary filter
%pitch --> ao redor do eixo y
%roll --> ao redor do eixo x
%yaw --> ao redor do eixo z
%
%BIAS:
%
%gx = 0.0334
%gy = 0.0016
%gz = 0.0078
%ax = 0.0778
%ay = 0.0152
%az = 0.0461
%Movimento 1 --> ROLL
%Movimento 2 --> PITCH
%Movimento 3 --> Yaw

%% SETUP
%Nessa parte está sendo pegue os dados do arquivo, além de retirar o bias
clear;
clc;

nome = input('Digite o nome do arquivo que deseja ler: ','s');

nome = strcat(nome,'.csv');

data = csvread(nome,2);
accx = (data(:,4));%-0.0778;
accy = (data(:,5));%-0.0152;
accz = (data(:,6));%-0.0461;

gyrox = data(:,1)*pi/180;
gyroy = data(:,2)*pi/180;
gyroz = data(:,3)*pi/180;

time = data(:,7)/1000000;

%Bias
gyrox = (gyrox);%- 0.0334;
gyroy = (gyroy);%- 0.0016;
gyroz = (gyroz);%- 0.0078;

%%{
tol = 0.001;
bias_gx = 0;
bias_gy = 0;
bias_gz = 0;

bias_ax = 0;
bias_ay = 0;
bias_az = 0;
i = 0;
ax = 0;
ay = 0;
az = 0;
gx = 0;
gy = 0;
gz = 0;
while(i == 0|| abs(bias_gx) > tol || abs(bias_gy) > tol || abs(bias_gz) > tol || abs(bias_ax) > tol || abs(bias_ay) > tol || abs(bias_az) > tol)
    gyrox = gyrox + bias_gx;
    gyroy = gyroy + bias_gy;
    gyroz = gyroz + bias_gz;
    
    accx = accx + bias_ax;
    accy = accy + bias_ay;
    accz = accz + bias_az;
    
meangyrox = sum(gyrox(1:1000))/1000;
meangyroy = sum(gyroy(1:1000))/1000;
meangyroz = sum(gyroz(1:1000))/1000;

meanaccx = sum(accx(1:1000))/1000;
meanaccy = sum(accy(1:1000))/1000;
meanaccz = sum(accz(1:1000))/1000;

%if(abs(bias_gx) > tol || i == 0)
bias_gx = 0 - meangyrox;
    gx = gx + abs(bias_gx);
%end
%if(abs(bias_gy)>tol || i==0)    
bias_gy = 0 - meangyroy;
    gy = gy + abs(bias_gy);
%end
%if(abs(bias_gz)>tol || i==0)
bias_gz = 0 - meangyroz;
    gz = gz + abs(bias_gz);
%end

%if(abs(bias_ax)>tol || i==0)
bias_ax = 0 - meanaccx;
    ax = ax + abs(bias_ax);
%end
%if(abs(bias_ay)>tol || i==0)
bias_ay = 0 - meanaccy;
    ay = ay + abs(bias_ay);
%end
%if(abs(bias_az)>tol || i==0)
bias_az = 1 - meanaccz;
    az = az + abs(bias_az);
%end
i = i+1;

end
ax
ay
az
gx
gy
gz
return
%}

Ts = 1/200; %Período, verificar a frequência depois

%% Angle for gyro
%Nessa parte iremos determinar o ângulo do giroscópio a partir dos dados
%recebidos.
%{
angy(1) = 0;
angx(1) = 0;
for i=1:length(gyroy)
   if i<length(gyroy)
        angy(i+1) = angy(i) + gyroy(i+1)*(time(i+1)-time(i));
        angx(i+1) = angx(i) + gyrox(i+1)*(time(i+1)-time(i));
   end
end
%}
%{
% You can use this part if you want to see the graph the gyro

plot(angy,'-');
hold on;
plot(angx,'-g');
title('Gyroscope without filter');
legend('Angy','Angx');
   return
%}

%% HIGH PASS FILTER
   %{
    % FC = 1/(2*pi*tau)
   % Devemos escolher uma frequência de corte
    FC = 10;
    tau = 1/(2*pi*FC);
   % O filtro passa alta trabalha com essa natureza:
    hpf = tau/(tau + Ts);

   gyroyhp(1) = angy(1);
   gyroxhp(1) = angx(1);
   
   %alphp = 0.8641;
   
    for i=2:length(gyrox)
        gyroyhp(i) = (hpf*gyroyhp(i-1)+hpf*(angy(i)-angy(i-1)));
        gyroxhp(i) = (hpf*gyroxhp(i-1)+hpf*(angx(i)-angx(i-1)));
    end
    %}
 %{
   %If you want to show the graphs from gyro
    subplot(1,2,1);
    plot(angy);
    hold on;
    plot(gyroyhp,'k-');
    legend('Angle using raw gyro','Angle passed through HPF');
    
    subplot(1,2,2);
    plot(angx);
    hold on;
    plot(gyroxhp,'g-');
    legend('Angle using raw gyro','Angle passed through HPF');
    return;
 %}   
%% Angle from Accelerometer
    for i=1:length(accx)
     acc_p(i) = atan2(accx(i),sqrt((accy(i)*accy(i))+(accz(i)*accz(i))));
     acc_r(i) = atan2(accy(i),sqrt((accx(i)*accx (i))+(accz(i)*accz(i))));
    end
   
%Plotagem acc e velocidade angular
  %{
    plot(acc_r)
    hold on;
    plot(gyrox)
    legend('Accel x','w_roll');
    return
    %}
 %% LOW PASS FILTER
   
 %{
    %FC = 1/(2*pi*tau);
    FC = 3;
    tau = 1/(2*pi*FC);
    lpf = 1/(Ts+1)
 
    acc_pitch(1) = 0;
    acc_roll(1) = 0;
   
    for i=2:length(accx)
        acc_pitch(i) = lpf*acc_p(i) + (1-lpf)*acc_pitch(i-1);
        acc_roll(i) = lpf*acc_r(i) + (1-lpf)*acc_roll(i-1);
    end
   %}
 %{
  %If you want to show the data from accelerometer
  subplot(1,2,1); 
  plot(acc_p,'r-');
  hold on;
  plot(acc_pitch,'g-');
  legend('Accel Pitch','Angle passed through LPF');
 
  subplot(1,2,2);
  plot(acc_r,'r-');
  hold on;
  plot(acc_roll,'g-');
  legend('Accel Roll','Angle passed through LPF');
   
   return;
  %} 

%% Complementary Filter  
   gyro_x(1) = 0;
   gyro_y(1) = 0;
      
   cf_roll = zeros(length(accx),1);
   cf_pitch = zeros(length(accx),1);
   
   cf_roll(1) = 0;
   cf_pitch(1) = 0;
   
   for i = 2:length(accx)
      gyro_y(i) = (gyro_y(i-1)+gyroy(i-1)*1/176); 
      gyro_x(i) =(gyro_x(i-1)+gyrox(i-1)*1/176);
      
      a = 0.98;
      
      cf_pitch(i) = a*(cf_pitch(i-1)+gyro_y(i)-gyro_y(i-1))+(1-a)*acc_p(i);  
      cf_roll(i) = a*(cf_roll(i-1)+gyro_x(i)-gyro_x(i-1))+(1-a)*acc_r(i);  
   end
   
   %%{
   subplot(1,2,1);
   plot(gyro_y);
   hold on;
   plot(acc_p,'g');
   plot(cf_pitch,'r');
  %ylim([-pi/2 pi/2]);
   legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('PITCH');
   
   subplot(1,2,2);
   plot(gyro_x);
   hold on;
   plot(acc_r,'g');
   plot(cf_roll,'r');
   %ylim([-pi/2 pi/2]);
   legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('ROLL');
   %}
