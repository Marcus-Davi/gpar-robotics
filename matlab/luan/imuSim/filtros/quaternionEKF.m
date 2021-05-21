classdef quaternionEKF < handle

    properties
        x = [1;0;0;0];
        dt = 0;
        g = 9.5;
        P = 0.1*eye(4);
        R = 0;
        Q = 0;
       %x = [q0;q1;q23;q3];
        
    end
    
    methods
        function obj = quaternionEKF(dt, gyr_noise, gyr_var)
            obj.dt=dt;
            obj.Q = diag(gyr_noise);
            obj.R = diag(gyr_var);
        end
        
        function x_pred = predict(obj,w )
            
            
            obj.x = [obj.x(1) + obj.dt/2*(-obj.x(2)*w(1) -obj.x(3)*w(2) -obj.x(4)*w(3));
                     obj.x(2) + obj.dt/2*( obj.x(1)*w(1) -obj.x(4)*w(2) +obj.x(3)*w(3));
                     obj.x(3) + obj.dt/2*( obj.x(4)*w(1) +obj.x(1)*w(2) -obj.x(2)*w(3));
                     obj.x(4) + obj.dt/2*(-obj.x(3)*w(1) +obj.x(2)*w(2) -obj.x(1)*w(3))];
            
           J = [2/obj.dt  -w(1)    -w(2)     -w(3);
                  w(1)   2/obj.dt   w(3)     -w(2);
                  w(2)    -w(3)   2/obj.dt    w(1);
                  w(3)    w(2)     -w(1)    2/obj.dt] * obj.dt/2;
                     
            obj.P = J* obj.P *J' + obj.Q;

            x_pred = obj.x;

        end
        
        function x = update(obj, g_measured)
            y = [0 ; g_measured];
            y_hat = obj.g * [0;
                             2*(obj.x(4)*obj.x(2)-obj.x(3)*obj.x(1));
                             2*(obj.x(4)*obj.x(3)+obj.x(2)*obj.x(1));
                             obj.x(4)*obj.x(4)-obj.x(3)*obj.x(3)-obj.x(2)*obj.x(2)+obj.x(1)*obj.x(1)];
            
             H = 2 *obj.g* [     0         0         0        0 ;
                            -obj.x(3)  obj.x(4) -obj.x(1) obj.x(2);
                             obj.x(2)  obj.x(1)  obj.x(4) obj.x(3);
                             obj.x(1) -obj.x(2) -obj.x(3) obj.x(4)];
            
            e = y - y_hat
            K = obj.P* H'*inv(H * obj.P * H' + obj.R);
            obj.x = obj.x + K*e;
            obj.P = obj.P - K*H*obj.P;
            
            x = obj.x;
        end
    end
end

