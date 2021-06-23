classdef quaternionESKF < handle

    properties
        x = [1;0;0;0;0;0;0;0;0;9.81];
        dt = 0;
        P = 0.1*eye(9);
        R = 0;
        Q = 0;
       %x = [q0;q1;q23;q3];
        
    end
    
    methods
        function obj = quaternionESKF(dt, gyr_noise, gyr_var, acc_var)
            obj.dt=dt;
            obj.Q = blkdiag(diag(gyr_var), diag(gyr_noise), 0*eye(3));
            
            obj.R = diag([1; acc_var]); 
        end
        
        function x_pred = predict(obj,wm )
            wb = obj.x(5:7);
            q  = obj.x(1:4); 
            
            w = [0;(wm-wb)];
            q = q + obj.dt/2*quatmultiply(q',w')';
            obj.x(1:4) = q; 
            
           F = [quat2rotm(w*obj.dt) -obj.dt*eye(3)  0*eye(3);
                0*eye(3)                eye(3)      0*eye(3);
                0*eye(3)              0*eye(3)        eye(3)];
                     
            obj.P = F* obj.P *F' + obj.Q;

            x_pred = obj.x(1:4);

        end
        
        function x = update(obj, g_measured)
            q  = obj.x(1:4);
            y = [0 ; g_measured];
            
            y_hat = quatrotate(q, [0;0;obj.g]')';
            y_hat = [0;y_hat];
            
            Hx = 2 *obj.g* [     0         0         0        0 ;
                            -obj.x(3)  obj.x(4) -obj.x(1) obj.x(2);
                             obj.x(2)  obj.x(1)  obj.x(4) obj.x(3);
                             obj.x(1) -obj.x(2) -obj.x(3) obj.x(4)];
            
                         
            qdq= 1/2*[-q(2) -q(3) -q(4);
                       q(1) -q(4)  q(3);
                       q(4)  q(1) -q(2);
                       q(3)  q(2)  q(1)];
            Hdx = blkdiag(qdq, eye(3), eye(3));
            H = Hx*Hdx;
            
            e = y - y_hat; %#ok<NOPRT>
            K = obj.P* H'*inv(H * obj.P * H' + obj.R); %#ok<MINV>
            dx = K*(e - obj.dx);
            obj.P = obj.P - K*H*obj.P;
            
            obj.x(1:4) = quatmultiply(obj.x(1:4)', dx')';
            x = obj.x(1:4);
        end
    end
end
