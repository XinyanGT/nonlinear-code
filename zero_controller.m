classdef zero_controller < handle
    properties
        sys
    end
    
    methods
        function obj = zero_controller(sys)
            obj.sys = sys;
        end
        
        function u = control(obj, t, q)
            % Unpack urrent state members
            % You could read data in sys, like goal, and time span, and use
            % the functions in sys
            [x, v, varphi, omega] = obj.sys.getStateMember(q);

            % Your control control input is computed here
            u = zeros(4,1);   
            
            % You may want to save some statistics of the controllers to
            % data members
            
            
        end
    end

end

