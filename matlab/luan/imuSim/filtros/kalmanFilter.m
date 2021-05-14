classdef kalmanFilter < handle
    %KALMANFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; %identidade
        C = [1 0 0 0; 0 0 1 0]; 
        A = 0;
        B = 0;
        Q = 0;
        R = 0;
        x_kf = 0;
        x_ = 0;
    end
    
    methods
        function obj = kalmanFilter(dt,gyrx_bias, gyry_bias, accx_var, accy_var)
            %KALMANFILTER Construct an instance of this class
            %   Detailed explanation goes here
            obj.R = [accx_var, 0; 0, accy_var];
            obj.A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
            obj.B = [dt 0; 0 0; 0 dt; 0 0];
            obj.x_kf = [0; gyrx_bias; 0; gyry_bias];
            obj.Q = 0.01*[dt*dt/2 0 0 0; 0 dt 0 0; 0 0 dt*dt/2 0; 0 0 0 dt];
        end
        
        function r = predict(obj,w_r, w_p)
            u = [ w_r; w_p];
            obj.x_ = obj.A*obj.x_kf + obj.B*u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
            r = obj.x_;
        end
        function r = update(obj, acc_roll, acc_pitch)
            
            y = [acc_roll; acc_pitch];
            K = obj.P * obj.C'/(obj.C * obj.P * obj.C' + obj.R);
            obj.x_kf = obj.x_ + K*( y - obj.C*obj.x_);
            obj.P = obj.P - K*obj.C*obj.P;
            r = obj.x_kf;
        end
        
        function obj = set.P(obj, p)
            obj.P = p;
        end
            
        function obj = set.x_(obj, x)
            obj.x_ = x;
        end
        function obj = set.x_kf(obj, x)
            obj.x_kf = x;
        end
    end
end

