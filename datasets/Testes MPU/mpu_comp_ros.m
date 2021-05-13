%% Descrição do Código
%{
Autor: William Santos
Ano: 2020

Objetivo: Se inscrever em um tópico do ROS, e receber os dados do MPU a
partir desse tópico. Esses dados serão usados para a aplicação do
complementary filter.

Configurações necessárias:
- No início do código temos a variável freq, ela é importante para o
cálculo do ângulo do giroscópio. Altere o valor dessa variável de acordo
com a frequência dos dados recebidos do mpu.

Funções utilizadas:
Nesse código, é utilizada a função -->
- bias_calculation
    Nessa função, ela utiliza também outra função chamada "dados_mpu" e
    "read_mpu_arq"

Como utilizar o código:
- A função mpu_comp_ros não retorna nenhuma variável, mas precisa de 4
Parâmetros para poder funcionar.
- ROS_MASTER_IP: indicar o IP de quem está rodando o ROSCORE.
- TOPIC: nome do tópico que queremos nos inscrever
- tam: Indica a quantidade de amostras que serão lidas para fazer a
calibração do mpu
- n: Se colocado 1, o BIAS será calculado a partir de um arquivo csv. Se
colocado 0, então iremos receber os "tam" dados do MPU, por isso, se
escolhido essa opção, deixe o seu MPU em repouso para que possa ser feita a
devida calibração do componente. 

Código em execução:
- Com o código sendo executado, será mostrado na tela dois gráficos. Em
cada gráfico teremos 3 curvas, essas curvas representam os dados do
acelerômetro, do giroscópio e do filtro complementar. Esses dados são as
últimas 100 amostras recebidas pelo sensor MPU.
%}
function mpu_comp_ros(ROS_MASTER_IP,TOPIC,tam,n)
%% SETUP
rosshutdown;
clc;
rosinit(ROS_MASTER_IP)

% BIAS Calculation
    %Inicialmente, deixe o seu MPU parado para a coleta de dados para a sua
    %calibração. Usar n = 1, para leitura de um arquivo ou n = 0 para
    %leitura em tempo real dos dados do sensor MPU. 
    
    %PRECISO AJEITAR A PARTE DA CALIBRAÇÃO
    display('Coletando dados para a calibração');
    [b_ax,b_ay,b_az,b_gx,b_gy,b_gz] = bias_calculation(ROS_MASTER_IP,TOPIC,tam,n);
    display('Calibração calculada');
    
sub = rossubscriber(TOPIC);
msg = rosmessage(sub);

gyro_x(1) = 0;
gyro_y(1) = 0;
gyro_z(1) = 0;

cf_pitch(1) = 0;
cf_roll(1) = 0;

acc_p = 0;
acc_r = 0;

i = 1;
k = 0;

freq = 100;

%% LOOP
    %As variáveis que demarcadas com a posição (1) armazenam valores passados, e os valores
    %atuais são salvos no "slot" (2)
while(1)
   msg = receive(sub);
   %% Aquisição de dados
   accx = msg.LinearAcceleration.X - b_ax;
   accy = msg.LinearAcceleration.Y - b_ay;
   accz = msg.LinearAcceleration.Z - b_az;
   
   gyrox(2) = msg.AngularVelocity.X - b_gx;
   gyroy(2) = msg.AngularVelocity.Y - b_gy;
   gyroz(2) = msg.AngularVelocity.Z - b_gz;
   
   %% Calculando ângulos
      gyro_y(2) = (gyro_y(1)+gyroy(1)*1/freq); 
      gyro_x(2) =(gyro_x(1)+gyrox(1)*1/freq);
      
      acc_p = -atan2(accx,accz);
      acc_r = atan2(accy,sqrt((accx*accx)+(accz*accz)));
      
      a = 0.98;
      
      cf_pitch(2) = a*(cf_pitch(1)+gyro_y(2)-gyro_y(1))+(1-a)*acc_p;  
      cf_roll(2) = a*(cf_roll(1)+gyro_x(2)-gyro_x(1))+(1-a)*acc_r;  
  
   %% Armazenando valores que serão plotados
   pitch(i) = cf_pitch(1);
   roll(i) = cf_roll(1);
   gx(i) = gyro_x(1);
   gy(i) = gyro_y(1);
   ap(i) = acc_p(1);
   ar(i) = acc_r(1);
   
   %% Armazenando os valores passados
   gyrox(1) = gyrox(2); 
   gyroy(1) = gyroy(2);
   gyroz(1) = gyroz(2);
   
   gyro_x(1) = gyro_x(2);
   gyro_y(1) = gyro_y(2);
   
   cf_pitch(1) = cf_pitch(2);
   cf_roll(1) = cf_roll(2);
   
   %% Plotagem
   if(i == 50)
       if(k == 1)
        delete(p);
        delete(r);
        delete(ap_p);
        delete(gx_p);
        delete(ar_p);
        delete(gy_p);
        k = 0;
       end    
   subplot(1,2,1);
   gy_p = plot(gy,'-r');
   hold on;
   ap_p = plot(ap,'-g');
   p = plot(pitch,'-b');
   ylim([-pi/2 pi/2]);
   legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('PITCH');
   
   subplot(1,2,2);
   gx_p = plot(gx,'-r');
   hold on;
   ar_p = plot(ar,'-g');
   r = plot(roll,'-b');
   ylim([-pi/2 pi/2]);
   legend('Gyro Angle','Accel_Angle','Complementary Filter');
   title('ROLL');    
      i = 0;
      k = 1;
   end
 i = i+1;  
end
end
