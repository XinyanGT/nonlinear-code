function run()
    
    % System
    sys = quadrotor();
    
    % Controller
%     con = zero_controller(sys);
    con = constant_controller(sys, [40, 0.01, 0.02, 0]');
    
    % Set end conditions for sys
%     x0 = [0, 0, 10]; varphi0 = [0.2, 0, 0]; v0 = [4, 0, 0]; omega0 = [0, 0, 0.3];   % initial
    x0 = [0, 0, 0]; varphi0 = [0, 0, 0]; v0 = [0, 0, 0]; omega0 = [0, 0, 0];   % initial
    xT = [0, 0, 0]; varphiT = [0, 0, 0]; vT = [0, 0, 0]; omegaT = [0, 0, 0];        % goal
    q0 = sys.constructState(x0, v0, varphi0, omega0); % pack state members to state
    qT = sys.constructState(xT, vT, varphiT, omegaT);    
    tSpan = [0, 6]; % time span of simulation
    sys.setEnds(q0, qT, tSpan); % set in sys
    
    % Simulate with a specific controller
%     simulate(sys, @PD_controller);
    [t, q, u] = simulate(sys, con);
    
    % Visualization
    visualize(sys, t, q, u);
end