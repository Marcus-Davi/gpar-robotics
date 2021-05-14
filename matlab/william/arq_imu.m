% Programa para gravar um arquivo do meu IMU parado

function [accx accy accz gyrox gyroy gyroz] = arq_imu(escolha);
    %escolha = 2 --> escrita movimento
    %escolha = 1 --> escrita parado
    %escolha = 0 --> leitura
    
    addpath('./movimentos');
    
    if(escolha == 1)
        [accx ,accy , accz , gyrox , gyroy , gyroz] = imu(1000,0);
    elseif (escolha == 2)
        [accx ,accy , accz , gyrox , gyroy , gyroz] = imu(1000,1);
    end
    
    if(escolha == 1 || escolha == 2)
    matriz_mpu = [gyrox gyroy gyroz accx accy accz];
    nome = input('Digite o nome do arquivo: ','s');
    
    nome = strcat(nome,'.csv');
    
    csvwrite(nome,matriz_mpu)
    else
       nome = input('Digite o nome do arquivo que deseja ler: ','s');
       nome = strcat(nome,'.csv');
        
       data = csvread(nome);
       accx = data(:,4);
       accy = data(:,5);
       accz = data(:,6);

       gyrox = data(:,1);
       gyroy = data(:,2);
       gyroz = data(:,3); 
    end
    
end