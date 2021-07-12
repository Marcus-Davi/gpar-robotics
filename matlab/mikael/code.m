clc;
clear;
close all;

%% Importe data

movimento_filename = '../../datasets/simulation/movimento.csv';
parado_filename = '../../datasets/simulation/parado.csv';
ground_truth_filename = '../../datasets/simulation/ground_truth.csv';

% formato dos dados csv: [ax ay az gx gy gz mx my mz]
data = csvread(movimento_filename);
calib_data = csvread(parado_filename);
ground_truth = csvread(ground_truth_filename);

acc = [data(:,1) data(:,2) data(:,3)];
gyr = [data(:,4) data(:,5) data(:,6)];

%dados obtidos com sensor inerte
acc_calib = [calib_data(:,1) calib_data(:,2) calib_data(:,3)];
gyr_calib = [calib_data(:,4) calib_data(:,5) calib_data(:,6)];

acc_caluib_mean = mean(acc_calib);
gyr_calib_mean = mean(gyr_calib); %bias

gyr_calibrado = gyr - gyr_calib_mean; %remove bias
acc_calibrado = acc;

%% Calculos
% alfa - roll, beta - pitch, gama - yaw
syms alfa beta gama
Ra = [  1       0       0; 
        0   cos(alfa)   -sin(alfa); 
        0   sin(alfa)   cos(alfa)]
Rb = [  cos(beta)   0           sin(beta);
        1           0           0;
        0           sin(beta)   cos(beta)]
    
Rc = [  cos(alfa)   -sin(alfa)  0; 
        sin(alfa)   cos(alfa)   0;
        0           0           1; ]

teta = acc_calibrado(:,1)./acc_calibrado(:,2);
