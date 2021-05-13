%% Descrição do código
%{
Autor: William Santos
Ano: 2020

Objetivo: 
Ler os dados de um tópico ROS, na qual tem as informações do MPU
(giroscópio e acelerômetro). Importante dizer que a mensagem do tópico
precisa ser do tipo <sensor_msgs/Imu> para esse código funcionar. 

Configurações necessárias:
---

Funções utilizadas:
---

Como utilizar o código:
Nesse código temos 4 parâmetros
- ROS_MASTER_IP: indica o ip de quem está a rodar o ROSCORE
- TOPIC: Tópico que o código irá se inscrever para receber as mensagens
- tam: Indica quantos dados serão lidos. Esses dados serão armazenados em
variáveis que a função retorna.
- n: Se 2, então queremos iniciar a conexão ao roscore mencionado. Se
diferente de 2, indica que nossa conexão já foi estabelecida e agora só
queremos realizar a leitura de "tam" dados.

Código em execução:
O código dependendo da escolha de n, pode ou não iniciar a conexão com o
roscore. Essa possibilidade é importante, pois podemos chamar essa função
em outro código que já se conectou com o roscore. 
O código se inscreve no tópico e vai para o loop.O loop é executado "tam"
vezes, na qual em cada execução é salvo os valores lidos nas variáveis:
"accx accy accz gyrox gyroy gyroz". Esses valores são retornados pela
função.
Além disso, podemos salvar os dados lidos. É só escrever "Y" ou "y" para
salvar os dados. Que são salvos nessa ordem:

"gyrox gyroy gyroz accx accy accz"

%}
function [accx accy accz gyrox gyroy gyroz] = dados_mpu(ROS_MASTER_IP,TOPIC,tam,n)
%% SETUP
if(n==2)
rosshutdown;
clc;
rosinit(ROS_MASTER_IP)
end
sub = rossubscriber(TOPIC);
msg = rosmessage(sub);

i = 1;
%% LOOP
while(i<=tam)
    display('Amostras coletadas: ');
    display(i);
    
    msg = receive(sub);
   
   %Aquisição de dados
   accx(i,1) = msg.LinearAcceleration.X;
   accy(i,1) = msg.LinearAcceleration.Y;
   accz(i,1) = msg.LinearAcceleration.Z;
   
   gyrox(i,1) = msg.AngularVelocity.X;
   gyroy(i,1) = msg.AngularVelocity.Y;
   gyroz(i,1) = msg.AngularVelocity.Z;
   
   i = i+1;
end
escolha = input('Would you like to save this data? Y or N: ','s');

if(escolha == 'y'|| escolha == 'Y')
    matriz_mpu = [gyrox gyroy gyroz accx accy accz];
    nome = input('Digite o nome do arquivo: ','s');
    
    nome = strcat(nome,'.csv');
    
    csvwrite(nome,matriz_mpu);
end
end
