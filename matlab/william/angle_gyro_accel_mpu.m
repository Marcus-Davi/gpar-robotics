%% Descrição do código
%{
  Autor: William Santos
  Ano: 2020

  Objetivo do código:
   - Calcular os ângulos pitch e roll, para os códigos do acelerômetro e
   giroscópio

  Configurações necessárias:
    - Alterar a variável "freq" de acordo com a frequência da coleta dos
    dados realizados. 

  Funções utilizadas:
    ---

  Como utilizar o código:
    - Basta indicar os valores de accx accy accz gyrox gyroy gyroz e com
    isso o código irá calcular os valores dos ângulos e retornar esses
    valores ao usuário. 

%}
function [acc_p acc_r gyro_y gyro_x] = angle_gyro_accel_mpu(accx,accy,accz,gyrox,gyroy,gyroz)
    freq = 100;
    
%% Angle from Accelerometer and Gyroscope
    acc_p = zeros(length(accx),1);
    acc_r = zeros(length(accy),1);
    gyro_x = zeros(length(gyrox),1);
    gyro_y = zeros(length(gyroy),1);
    
    acc_p(1,1) = -atan2(accx(1),sqrt(accz(1)*accz(1)+accy(1)*accy(1)));
    acc_r(1,1) = atan2(accy(1),sqrt(accz(1)*accz(1)+accx(1)*accx(1)));
    
    gyro_x(1,1) = 0;
    gyro_y(1,1) = 0; 
    
for i=2:length(accx)
    gyro_y(i) = (gyro_y(i-1)+gyroy(i-1)*1/freq); 
    gyro_x(i) =(gyro_x(i-1)+gyrox(i-1)*1/freq);
    acc_r(i) = atan2(accy(i),sqrt(accz(i)*accz(i)+accx(i)*accx(i)));
    acc_p(i) = -atan2(accx(i),sqrt(accz(i)*accz(i)+accy(i)*accy(i)));
end

end