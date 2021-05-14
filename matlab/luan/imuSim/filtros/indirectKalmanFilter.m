classdef indirectKalmanFilter < handle
    %INDIRECTKALMANFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        C = [1 0 0 0; 0 0 1 0];
        A = 0;
        B = 0;
        Q = 0;
        R = 0;
        x = 0;
    end
    
    methods
        function obj = indirectKalmanFilter(dt, gyrx_var, gyry_var, accx_var, accy_var, gyrx_bias, gyry_bias)
            %INDIRECTKALMANFILTER Construct an instance of this class
            %   Detailed explanation goes here
            obj.A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
            obj.Q = 0.01*[dt*dt/2 0 0 0; 0 dt 0 0; 0 0 dt*dt/2 0; 0 0 0 dt];
            obj.R = [accx_var,0; 0, accy_var];
            obj.B = [dt 0; 0 0; 0 dt; 0 0];
            obj.x = [0;gyrx_bias;0;gyry_bias];
        end
        
        function pos = predict(obj, w_x, w_y)
            u = [w_x;w_y];
            obj.x = obj.A*obj.x + obj.B*u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
            pos = obj.x;
        end
        function pos = update(obj, acc_roll, acc_pitch)
            y = [acc_roll; acc_pitch];
            K = obj.P * obj.C'/(obj.C * obj.P * obj.C' + obj.R);
            erro = K*(y - obj.C*obj.x);
            obj.x = obj.x + erro;
            obj.P = (eye(4) - K*obj.C)*obj.P;
            pos = obj.x;
            
        end
    end
end

