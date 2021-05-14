%% Descrição do código
%{
Autor: William Santos
Ano: 2020

Objetivo do código:
- Realizar o filtro complementar dos dados do giroscópio e do acelerômetro. 

Configurações necessárias:
- Você pode alterar o valor de "a" se desejado, esse valor indica a
contribuição do giroscópio e do acelerômetro para o filtro. Você pode
alterar esse valor até estar em um nível adequado. 

Funções Utilizadas: 
- Utilizamos 3 funções nesse código
    1. read_mpu_arq (Responsável por ler os dados de um arquivo csv, que
    contenha os dados do giroscópio e acelerômetro)
    2. bias_calculation (Responsável por ler os dados de um arquivo csv e
    calcular o bias, recomendável esse arquivo ser do MPU em repouso)
    3. angle_gyro_accel_mpu (A partir dos dados coletados, e feito os
    devidos ajustes com o bias, esses dados serão enviados para essa função
    que calcula os ângulos do giroscópio e do acelerômetro).

Como utilizar o código:
    Não precisamos de nenhum parâmetro para a execução do código. No
    momento de execução, precisaremos indicar dois arquivos
    1. O primeiro arquivo é para a leitura dos dados do mpu, do movimento
    que fizemos com o componente, ou até do mesmo em repouso.
    2. O segundo arquivo que indicamos é o que será usado para calcular o
    bias.
    Com essas duas informações, o código continua e plota, ao final, os
    dados do giroscópio, do acelerômetro e do filtro complementar, em dois
    gráficos, um representando o ROLL e o outro o PITCH. 

Importante indicar que ROLL é o giro ao redor do eixo x. Enquanto o PITCH é
o giro ao redor do eixo y. E por fim, o YAW é o giro ao redor do eixo Z.
%}
function [cf_pitch,cf_roll,gyro_y,gyro_x,acc_p,acc_r,accx,accy,accz,gyrox,gyroy,gyroz] = complementary_filter()
clc;
%% SETUP
    display('O próximo arquivo é para leitura dos dados');
[accx accy accz gyrox gyroy gyroz] = read_mpu_arq();

    display('O próximo arquivo é para a calibração');
[accx_0,accy_0,accz_0,gyrox_0,gyroy_0,gyroz_0] = bias_calculation(0,0,0,1);

%% Calibração

accx = accx + accx_0;
accy = accy + accy_0;
accz = accz + accz_0;

gyrox = gyrox + gyrox_0;
gyroy = gyroy + gyroy_0;
gyroz = gyroz + gyroz_0;

%% Ângulos
freq = 100;

[acc_p, acc_r , gyro_y, gyro_x] = angle_gyro_accel_mpu(accx,accy,accz,gyrox,gyroy,gyroz);

%[acc_p acc_r] = linear_aceleration(gyrox,gyroy,gyroz,accx,accy,accz,acc_p,acc_r);

%% Complementary Filter
  
   cf_roll = zeros(length(accx),1);
   cf_pitch = zeros(length(accx),1);
   
   a = 0.98;
   
   for i = 2:length(accx)
      cf_pitch(i) = a*(cf_pitch(i-1)+gyro_y(i)-gyro_y(i-1))+(1-a)*acc_p(i);  
      cf_roll(i) = a*(cf_roll(i-1)+gyro_x(i)-gyro_x(i-1))+(1-a)*acc_r(i);  
   end
   
%% Plotagem
   %%{
   subplot(1,2,1);
   %plot(gyro_y,'-b');
   hold on;
   %plot(acc_p,'-g');
   plot(cf_pitch,'-g');
  ylim([-1.7 1.7]);
  % legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('PITCH');
   hold off;
   subplot(1,2,2);
   
  % plot(gyro_x,'-b');
   hold on;
  % plot(acc_r,'-g');
   plot(cf_roll,'-g');
   ylim([-1.7 1.7]);
   %legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('ROLL');
   hold off;
%}

end

