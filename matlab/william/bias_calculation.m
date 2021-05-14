%% Descrição do código
%{
Autor: William Santos
Ano: 2020

Objetivo: Receber determinada quantidade de dados do mpu, dados esses do
giroscópio e acelerômetro em repouso, para assim poder determinar o bias.
Essa bias terá como intuito deixar gx,gy,gz,ax e ay com leituras zero no
estado de repouso e az com o valor de 9,8. 

Configurações Necessárias:
O código não precisa de nenhum ajuste de variável ou coisa relacionado,
somente com os parâmetros da função o código funciona.

Funções utilizadas:
Para pegar os dados, esse código pode usa de duas funções
1. read_mpu_arq
2. dados_mpu

Como utilizar o código:
- A utilização do código se refere aos parâmetros passados, temos 4:
1.ROS_MASTER_IP indica o ip de quem está a executar o ROSCORE
2. TOPIC tópico que iremos nos inscrever caso a nossa coleta de dados seja
em tempo real
3. tam é a variável que indica quantas coletas serão feitas
4. n será a chave seletora, caso ele seja 1 o código coleta dados de um
arquivo salvo na mesma pasta do código, com essa opção, quando o código for
executado iremos precisar escrever o nome do arquivo que queremos ler. Se n
for 2 ele irá executar a coleta em tempo real na função "dados_mpu" e se
conectará com o roscore, porém, se você já estiver conectado com o roscore,
e quer apenas buscar os dados, então basta escolher n diferente de 1 e 2.
Se escolher a opção que utiliza a função "dados_mpu" deixe o seu mpu parado
para a coleta dos dados.

Código em execução:
Quando o código for executado, se escolher a leitura do arquivo, deverá
escrever o nome do arquivo a ser lido. Essa é a única ação possível durante
a execução do código.
Os valores que a função retorna são referentes ao dado do sensor já
calibrado.

%}
function [accx_0,accy_0,accz_0,gyrox_0,gyroy_0,gyroz_0] = bias_calculation(ROS_MASTER_IP,TOPIC,tam,n)
%% Variáveis
tol = 0.001;
bias_gx = 0;
bias_gy = 0;
bias_gz = 0;

bias_ax = 0;
bias_ay = 0;
bias_az = 0;

i = 0;

b_ax = 0;
b_ay = 0;
b_az = 0;
b_gx = 0;
b_gy = 0;
b_gz = 0;

%% Pegar dados
if(n==1)
[accx accy accz gyrox gyroy gyroz] = read_mpu_arq();
else
[accx accy accz gyrox gyroy gyroz] = dados_mpu(ROS_MASTER_IP,TOPIC,tam,n);
end

tam = length(accx);

b_gx = sum(gyrox(1:tam))/tam;
b_gy = sum(gyroy(1:tam))/tam;
b_gz = sum(gyroz(1:tam))/tam;

b_ax = sum(accx(1:tam))/tam;
b_ay = sum(accy(1:tam))/tam;
b_az = sum(accz(1:tam))/tam;

accx_0 = 0 - b_ax;
accy_0 = 0 - b_ay;
accz_0 = 9.8065 - b_az;
gyrox_0 = 0 - b_gx;
gyroy_0 = 0 - b_gy;
gyroz_0 = 0 - b_gz;
end