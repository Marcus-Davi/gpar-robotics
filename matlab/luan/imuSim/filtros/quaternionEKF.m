classdef quaternionEKF < handle

    properties
        x = [1;0;0;0];
        dt = 0;
        g = 9.8165;
        P = eye(4);
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
                  w(3)    w(2)     -w(1)    2/obj.dt];
                     
            obj.P = J'* obj.P *J' + obj.Q;

            obj.x = quat_normalize(obj.x(1),obj.x(2),obj.x(3),obj.x(4));
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
            
            e = y - y_hat;
            l = obj.P* H';
            alpha = H * obj.P * H' + obj.R;
            K = l/alpha
            obj.x = obj.x + K*e;
            obj.P = obj.P - K*l';
            
            obj.x = quat_normalize(obj.x(1),obj.x(2),obj.x(3),obj.x(4));
            x = obj.x;
        end
    end
end

function qnorm = quat_normalize(q1, q2, q3, q4)
    mod = sqrt(q1*q1 + q2*q2 + q3*q3 *q4*q4);
    if mod == 0
        qnorm = [0;0;0;0];
    else
        qnorm = [q1;q2;q3;q4] / mod;
    end
end
