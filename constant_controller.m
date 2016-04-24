classdef constant_controller < handle
    properties
        sys
        cons 
    end
    
    methods
        function obj = constant_controller(sys, cons)
            obj.sys = sys;
            obj.cons = cons;
        end
        
        function u = control(obj, t, q)
            % Unpack urrent state members
            % You could read data in sys, like goal, and time span, and use
            % the functions in sys
            [x, v, varphi, omega] = obj.sys.getStateMember(q);

            % Your control control input is computed here
            u = obj.cons;
            
            % You may want to save some statistics of the controllers to
            % data members
            
            
        end
    end

end

