classdef quaternionESKF < handle

    properties
        x = [1;0;0;0];
        dx = [0;0;0;0];
        dt = 0;
        g = 9.5;
        P = 0.1*eye(4);
        R = 0;
        Q = 0;
       %x = [q0;q1;q23;q3];
        
    end
    
    methods
        function obj = quaternionESKF(dt, gyr_noise, gyr_var)
            obj.dt=dt;
            obj.Q = diag(gyr_noise);
            obj.R = diag(gyr_var);
        end
        
        function x_pred = predict(obj,w )
            w = [0;w];
            
            obj.x = obj.x + obj.dt/2*quatmultiply(obj.x',w')';
            
           J = [2/obj.dt  -w(1)    -w(2)     -w(3);
                  w(1)   2/obj.dt   w(3)     -w(2);
                  w(2)    -w(3)   2/obj.dt    w(1);
                  w(3)    w(2)     -w(1)    2/obj.dt] * obj.dt/2;
                     
            obj.dx = J*obj.dx;
            obj.P = J* obj.P *J' + obj.Q;

            x_pred = obj.x;

        end
        
        function x = update(obj, g_measured)
            y = [0 ; g_measured];
            
            y_hat = quatrotate(obj.x', [0;0;obj.g]')';
            y_hat = [0;y_hat];
            
             H = 2 *obj.g* [     0         0         0        0 ;
                            -obj.x(3)  obj.x(4) -obj.x(1) obj.x(2);
                             obj.x(2)  obj.x(1)  obj.x(4) obj.x(3);
                             obj.x(1) -obj.x(2) -obj.x(3) obj.x(4)];
            
            e = y - y_hat; %#ok<NOPRT>
            K = obj.P* H'*inv(H * obj.P * H' + obj.R); %#ok<MINV>
            obj.dx = obj.dx + K*(e - obj.dx);
            obj.P = obj.P - K*H*obj.P;
            obj.x = obj.x-obj.dx;
            x = obj.x;
        end
    end
end
