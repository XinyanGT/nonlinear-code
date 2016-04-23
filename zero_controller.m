function u = zero_controller(sys, q)
    %% Goal 
    final = sys.final;
    
    %% Current state
    [x, v, varphi, omega] = sys.getStateMember(q);
    
    %% Control 
    u = zeros(4,1);
    
end

