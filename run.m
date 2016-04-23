function run()
    
    % System
    sys = quadrotor();
    
    % Set end conditions
    x0 = [0, 0, 10]; varphi0 = [0.2, 0, 0]; v0 = [4, 0, 0]; omega0 = [0, 0, 0.3];
    xT = [0, 0, 0]; varphiT = [0, 0, 0]; vT = [0, 0, 0]; omegaT = [0, 0, 0];    
    q0 = sys.constructState(x0, v0, varphi0, omega0);
    qT = sys.constructState(xT, vT, varphiT, omegaT);    
    tSpan = [0, 4];
    sys.setEnds(q0, qT, tSpan);
    
    % Simulate with a specific controller
%     simulate(sys, @PD_controller);
    [t, q, u] = simulate(sys, @zero_controller);
    
    % Visualization
    visualize(sys, t, q, u);
end