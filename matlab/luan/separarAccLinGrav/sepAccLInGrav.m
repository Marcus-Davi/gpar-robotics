%******************************************************************
% Autor: Luan R.
% 2021
%
% Programa que lê os dados do IMU e separa a aceleração 
% gravitacional da linear.
%******************************************************************
clc, clear, close
%***********  CALIBRAÇÃO *****************************************
 [accx_calib, accy_calib, accz_calib, gyrx_calib, gyry_calib, gyrz_calib] = read_mpu_arq();
 
 accx_bias = mean(accx_calib);
 accy_bias = mean(accy_calib);
 accz_bias = mean(accz_calib-9.8065);
 gyrx_bias = mean(gyrx_calib);
 gyry_bias = mean(gyry_calib);
 gyrz_bias = mean(gyrz_calib);
 
 accx_var = var(accx_calib);
 accy_var = var(accy_calib);
 accz_var = var(accz_calib-9.8065);
 gyrx_var = var(gyrx_calib);
 gyry_var = var(gyry_calib);
 gyrz_var = var(gyrz_calib);
 
 roll_var = var(atan2(accy_calib, sqrt(accx_calib.*accx_calib + accz_calib.*accz_calib)));
 pitch_var = var(-atan2(accx_calib, sqrt(accy_calib.*accy_calib + accz_calib.*accz_calib)));
 
 %*****************************************************************
 [accx, accy, accz, gyrx, gyry, gyrz] = read_mpu_arq();
 dataSize = length(accx);
 dt = 1/100;
 
%  accx = [0; accx];
%  accy = [0; accy];
%  accz = [0; accz];
%  gyrx = [0; gyrx];
%  gyry = [0; gyry];
%  gyrz = [0; gyrz];
 accx = [0; accx - accx_bias];
 accy = [0; accy - accy_bias];
 accz = [0; accz - accz_bias];
 gyrx = [0; gyrx - gyrx_bias];
 gyry = [0; gyry - gyry_bias];
 gyrz = [0; gyrz - gyrz_bias];
 
 % filtro passa baixa no gyr
for i=2:dataSize
   gyrx(i) = 0.02*gyrx(i) + 0.98*gyrx(i-1);
   gyry(i) = 0.02*gyry(i) + 0.98*gyry(i-1);
   gyrz(i) = 0.02*gyrz(i) + 0.98*gyrz(i-1);
end

 roll_acc  =  atan2(accy, sqrt(accx.*accx + accz.*accz));
 pitch_acc = -atan2(accx, sqrt(accy.*accy + accz.*accz));
 
 roll_gyr = zeros(dataSize +1,1);
 pitch_gyr = zeros(dataSize +1,1);
 
 accx_linear = zeros(dataSize+1,1);
 accy_linear = zeros(dataSize+1,1);
 accz_linear = zeros(dataSize+1,1);
 gyrx_linear = zeros(dataSize+1,1);
 gyry_linear = zeros(dataSize+1,1);
 gyrz_linear = zeros(dataSize+1,1);

 
 %pre processamento do filtro de kalman
 tol = 0.09;
for i=2:dataSize
   if(abs(gyry(i)) < tol)
      pitch_acc(i) = pitch_acc(i-1);
   end
   if(abs(gyrx(i)) < tol)
       roll_acc(i) = roll_acc(i-1);
   end
end


 for i=2:dataSize+1
    roll_gyr(i) = roll_gyr(i-1) + gyrx(i) * dt ;
    pitch_gyr(i) = pitch_gyr(i-1) + gyry(i) * dt;
 end    
 
 %********** KALMAN FILTER **********************
 P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; %identidade
 R = [gyrx_var 0; 0 gyry_var];
 C = [1 0 0 0; 0 0 1 0];
 B = [dt 0; 0 0; 0 dt; 0 0];
 Q = 0.01* [dt*dt/2 0 0 0; 0 dt+5 0 0; 0 0 dt*dt/2 0; 0 0 0 dt];
 A = [1 -dt 0 0 ; 0 1 0 0 ; 0 0 1 -dt; 0 0 0 1] ;
 
 roll_pred = zeros(dataSize+1, 1);
 pitch_pred= zeros(dataSize+1,1);
 roll_kf = zeros(dataSize+1,1);
 pitch_kf= zeros(dataSize+1,1);
  
 x_kf = [0; gyrx_bias; 0; gyry_bias];

 
 g = [0;0;9.8056];
 g_pred =  g;
 j = 0;
 for i=2:dataSize+1
     %predict
    x = A * x_kf + B*[gyrx(i); gyry(i)];
    P_pred = A * P * A' + Q;
    
    roll_pred(i) = x(1);
    pitch_pred(i)= x(3);

    %update
    y = [roll_acc(i); pitch_acc(i)];
    K = P * C' / (C * P * C' + R);
    x_kf = x + K*( y - C *x);
    P = P_pred - K*C*P_pred;
    
    roll_kf(i) = x_kf(1);
    pitch_kf(i) = x_kf(3);
    
 end
 %-------------------------------------------------------------------------
  %  passa baixa
  for i=2:dataSize 
%      roll_gyr(i) = 0.1*roll_gyr(i) + 0.9 * roll_gyr(i-1);
%      pitch_gyr(i) = 0.1*pitch_gyr(i) + 0.9*pitch_gyr(i-1);
     accx(i) = 0.1*accx(i) + 0.9* accx(i-1);
     accy(i) = 0.1*accy(i) + 0.9* accy(i-1);
  end
%  %-------------------------------------------------------------------------
 %corte de ruido
 for i=2:dataSize 
     if (abs(accx(i)) < 1*accx_bias)
        accx(i) = 0;
     end
      if (abs(accy(i)) < 1*accy_bias)
        accy(i) = 0;
     end
 end
%  %-------------------------------------------------------------------------

 %passa alta
 accyf= zeros(dataSize+1,1);
 accxf= zeros(dataSize+1,1);
 for i=2:dataSize
    accxf(i) = 0.9* (accx(i)- accx(i-1) + accxf(i-1));  
    accyf(i) = 0.9* (accy(i)- accy(i-1) + accyf(i-1));  

 end

 %-------------------------------------------------------------------------
 %Detecção de movimento Linear 
 k = 50;
 g = [0;0;9.8056];
 gB= [0;0;9.8056];
 
 pos = zeros(dataSize+1, 3);
 vx = zeros(dataSize+1, 1);
 vy = zeros(dataSize+1,1);
 
 acc_linear_gx = zeros(dataSize+1,1); 
 acc_linear_gy = zeros(dataSize+1,1); 
 
 nc_pitch = 0;
 nd_pitch = 0;

 for i = 2:dataSize+1   
     droll_gyr = roll_gyr(i) - roll_gyr(i-1);
     droll_kf  = roll_kf(i)  - roll_kf(i-1) ;
     dpitch_gyr = pitch_gyr(i) - pitch_gyr(i-1);
     dpitch_kf  = pitch_kf(i)  - pitch_kf(i-1) ;
    
     %alta correlação -> angular
     %baixa correlação -> linear
    
    if(abs(dpitch_gyr) <= k * abs(dpitch_kf))
       %movimento angular em x
    else
        %movimento linear em x
        dpitch_kf = 0;
    end

    if( abs(droll_gyr) <= k* abs(droll_kf) )
       %movimento angular em y
    else
        %movimento linear em y
       droll_kf = 0;
    end
    
    rot = roty(dpitch_kf) *rotx(droll_kf) ;
    gB = rot * gB;
    acc_linear_b = [ accxf(i); accyf(i); accz(i)] - gB;
    acc_linear_g = rot' * acc_linear_b;
    acc_linear_gx(i) = acc_linear_g(1);
    acc_linear_gy(i) = acc_linear_g(2);
    
    vx(i) = vx(i-1) + acc_linear_g(1)*dt;
    pos(i,1) = pos(i-1,1) + vx(i)*dt;
    vy(i) = vy(i-1) + acc_linear_g(2)*dt;
    pos(i,2) = pos(i-1,2) + vy(i)*dt ;
    
    
 end
 

% 
%  figure
%  grid on
%  hold on
%  plot(roll_kf, 'r')
%  plot(roll_acc,'g')
%  plot(roll_gyr,'b')
%  legend('kf','acc','gyr')
%  title('roll')
%  hold off
%  
%  figure
%  grid on
%  hold on
%  plot(pitch_kf, 'r')
%  plot(pitch_acc,'g')
%  plot(pitch_gyr,'b')
%  legend('kf','acc','gyr')
%  title('pitch')
%  hold off
 
figure
 subplot(2,3,1);
 grid on
 plot(pos(:,1))
 legend('pos x')
 title('posição')
 
subplot(2,3,4);
 grid on
 plot(10*pos(:,2)) 
 legend('pos y')
 title('posição')
 
subplot(2,3,3);
 grid on
 plot(acc_linear_gx)
 
 legend('acc linear gx')
title('acc linear x')
 
subplot(2,3,6);
 grid on
 plot(acc_linear_gy)
 legend('acc linear gy')
 title('aceleração y')

  
 subplot(2,3,2);
 grid on
 plot(vx)
 legend('vel x')
 title('velocidade')

subplot(2,3,5);
 grid on
 plot(vy)
 legend('vel y')
 title('velocidade')