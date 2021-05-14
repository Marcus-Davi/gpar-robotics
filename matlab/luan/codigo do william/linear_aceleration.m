% Tentativa de evitar leitura de angulos com movimentos lineares

function [acc_p acc_r] = linear_aceleration(gyrox,gyroy,gyroz,accx,accy,accz,acc_p,acc_r)

tam = length(accx);
faccx = zeros(tam,1);
faccy = zeros(tam,1);
faccz = zeros(tam,1);

fgyrox = zeros(tam,1);
fgyroy = zeros(tam,1);
fgyroz = zeros(tam,1);

a = 0.98;
tol_g = 0.09;          
        for i=2:tam
         fgyrox(i) = (1-a)*gyrox(i)+a*fgyrox(i-1);
         fgyroy(i) = (1-a)*gyroy(i)+a*fgyroy(i-1);
         fgyroz(i) = (1-a)*gyroy(i)+a*fgyroz(i-1);
        end
   p = 0;
   r = 0;
        for i=1:tam-1
           if(abs(fgyroy(i))<tol_g)
               acc_p(i) = p;
           else
               p = acc_p(i+1);
           end
           if(abs(fgyrox(i))<tol_g)
               acc_r(i) = r;
           else
               r = acc_r(i+1);
           end
        end
 
          
end
