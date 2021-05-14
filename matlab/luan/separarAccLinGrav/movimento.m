%programa que detecta se houve movimento

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
 
 accx = [0; accx - accx_bias];
 accy = [0; accy - accy_bias];
 accz = [0; accz - accz_bias];
 gyrx = [0; gyrx - gyrx_bias];
 gyry = [0; gyry - gyry_bias];
 gyrz = [0; gyrz - gyrz_bias];
 
 %filtro passa baixa
 for i=2:dataSize
    accx(i) = 0.05*accx(i) + 0.95*accx(i-1);
    accy(i) = 0.05*accy(i) + 0.95*accy(i-1);
    accz(i) = 0.05*accz(i) + 0.95*accz(i-1);     
 end
 
 
 
 moveu = zeros(dataSize+1,1);
 modulo = zeros(dataSize+1,1);
 tol = sqrt(accx_bias*accx_bias+ accy_bias*accy_bias+ accz_bias*accz_bias);
 for i=2:dataSize
     modulo = sqrt(accx(i)*accx(i) + accy(i)*accy(i)+ accz(i)*accz(i));
     if moveu(i-1) 
         ntol = tol/4;
     else
         ntol = tol/2;
     end
             
    if (abs(modulo-9.8056) < ntol) 
        %nao houve aumento de velocidade linear;
        moveu(i)=0;
        if(accx(i)-accx(i-1) > accx_bias/2 ||  accy(i)-accy(i-1) > accy_bias/2)
            moveu(i) = 1;
        end
    else 
        moveu(i)=1;
    end
 end
 
 figure
 hold on
 plot(moveu)
 plot(accy)
 plot(accx)
 hold off
 
 
 
 
 