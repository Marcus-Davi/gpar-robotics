function [flag] = flag_movimento(accx,accy,accz,flag)

g = 9.8065;
g_measured = sqrt(accx*accx+accy*accy+accz*accz);
t_up = 1.2;
t_down = -1.2;

tol = g_measured - g;

if(flag == 0)
    if(tol>=t_up)
        flag = 2;
    elseif(tol<=t_down)
        flag = -2;
    end
else
    if(flag == 1 && tol>=t_up)
        flag = 2;
    elseif(flag == -1 && tol<=t_down)
        flag = -2;
    elseif(flag == 2 && tol<=t_down)
        flag = 1;
    elseif(flag == -2 && tol>=t_up)
        flag = -1;
    end
end


end