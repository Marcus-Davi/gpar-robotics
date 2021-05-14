classdef quaternionKalmanFilter << handle
    
    properties
        x = [0;0;0;0;0;0;0];
       %x = [q0;q1;q23;q3;bx;by;bz];
        G = [zeros(4,3); eye(3)];
         
         B = 0;  
         P = eye();
         R = 0;         
        
        
    end
    
    methods
        function obj = quaternionKalmanFilter(gyr_noise,gyr_var)
            B = diag(gyr_noise);
            R = diag(gyr_var);
            
            
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function g_predicted = predict(obj,w_measure)
            
            x_predict = [(1/2)*q*w_measure - (1/2)*q*w_bias; b*w_bias];
            P = F* P *F' + Q;

        end
        
        function x = update(obj)
            
            f = P* H';
            alpha = H * f;
            K = f/alpha;
            x = x_predict* K* [y + h* x_predict];
            P = P - K*f';
       
        end
end

%gyroscope
x_dot = [(1/2)*q*w_measure - (1/2)*q*w_bias; beta*w_bias] + G*u;

f(x) = [(1/2)*q*w_measure - (1/2)*q*w_bias; b*w_bias];

x(t+1) = x(t) +f(x(t))*dt + G*u*dt;
