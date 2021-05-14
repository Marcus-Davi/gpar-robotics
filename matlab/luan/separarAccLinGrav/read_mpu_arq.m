
%% Descrição do código
%{
Autor: William Santos
Ano: 2020

Objetivo do código:
Ler os dados de um arquivo que armazenou dados do mpu6050. 

Configurações necessárias:
A estrutura do arquivo precisa estar padronizada da seguinte maneira -->
GYROX GYROY GYROZ ACCX ACCY ACCZ
Onde cada nome é uma coluna.
Se quiser que ele comece pegar os dados a partir de outra linha, sem ser a
primeira, basta alterar a seguinte parte do código:

data = csvread(nome,NÚMERO DA LINHA PARA ELE COMEÇAR A LER);

Outro ponto importante, arquivo tem que ser CSV.
Alguns pontos que você pode ajustar, é a ordem das colunas. É só alterar o
número, por padrão está:
GYROX --> 1
GYROY --> 2
GYROZ --> 3
ACCX --> 4
ACCY --> 5
ACCZ --> 6

Exemplo:
accx = (data(:,NUMERO DA COLUNA));

No código, você pode verificar esses números e alterá-los da maneira que
você salvou em seu arquivo.

Como utilizar o código: 
Para utilizar o código, não precisa de nenhum parâmetro sendo enviado para
ele. Só precisa escrever indicando o nome do arquivo a ser lido. O nome do
arquivo é sem a extensão, pois essa é adicionada depois.
O código irá ler o arquivo e retornar os dados
"accx,accy,accz,gyrox,gyroy,gyroz"
%}
function [accx accy accz gyrox gyroy gyroz] = read_mpu_arq();
nome = input('Digite o nome do arquivo que deseja ler: ','s');

nome = strcat(nome,'.csv');

data = csvread(nome);
accx = (data(:,4));
accy = (data(:,5));
accz = (data(:,6));

gyrox = data(:,1);
gyroy = data(:,2);
gyroz = data(:,3);

end