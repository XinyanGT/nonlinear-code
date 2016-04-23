classdef quadrotor < handle
    % A class of system
    properties
        g = 9.81;
        m = 4.34;
        d = 0.315;
        c_T = 1;
        c_Q = 8.004 * 1e-4;
        I = diag([0.0820, 0.0845, 0.1377]);
        indices = struct('x', 1:3, 'v', 4:6, 'varphi', 7:9, 'omega', 10:12);
        
        initial, final, tspan
    end
    
    methods
        %% Construct
        function obj = quadrotor()
        end
        
        function setEnds(obj, initial, final, tspan)
            obj.initial = initial; obj.final = final; obj.tspan = tspan;
        end            
        
        %% Dynamics
        function qdot = dynamics(obj, q, u)
            [x, v, varphi, omega] = obj.getStateMember(q);
            e3 = [0, 0, 1]';
            xdot = v;
            R = obj.getRotMat(varphi);
            vdot = -e3*obj.g -+1/obj.m*u(1)*R*e3;
            varphidot = inv(obj.getPhi(varphi)) * omega;
            omegadot = inv(obj.I) * (u(2:4) - ...
                                     obj.hat(omega)*obj.I*omega);
            qdot = obj.constructState(xdot, vdot, varphidot, omegadot);
        end        

        %% States related
        function [x, v, varphi, omega] = getStateMember(obj, q)
            % Get state members from a state
            idx = obj.indices;
            x = q(idx.x);
            v = q(idx.v);
            varphi = q(idx.varphi);
            omega = q(idx.omega);
        end
        

        function q = constructState(obj, x, v, varphi, omega)
            % Construct a state vector
            q = zeros(12, 1); idx = obj.indices;
            q(idx.x) = x; 
            q(idx.v) = v; 
            q(idx.varphi) = varphi;
            q(idx.omega) = omega;
        end
        
        %% Auxiliary
        function Phi = getPhi(obj, varphi)
            % Compute the matrix Phi that converts Euler angle rates to
            % angular velocity IN BODY FRAME
            phi = varphi(1); theta = varphi(2);
            cp = cos(phi); sp = sin(phi); st = sin(theta); ct = cos(theta);
            Phi = [1, 0, st; 0, cp, -sp*ct; 0, sp, cp*ct];
%             Phi = [0, cp, sp*st; 0, sp, -cp*st; 1, 0, ct];
        end
        
        function xhat = hat(obj, x) 
            % Compute the skew-symmetric matrix
           xhat =  [0, -x(3), x(2); ...
                    x(3), 0, -x(1); ...
                    -x(2), x(1), 0];
        end
        
        function R = getRotMat(obj, varphi)
            % Rotation matrix R01
            phi = varphi(1); theta = varphi(2); psi = varphi(3);
            c1 = cos(phi); c2 = cos(theta); c3 = cos(psi);
            s1 = sin(phi); s2 = sin(theta); s3 = sin(psi);
            R = [c2*c3, -c2*s3, s2; ...
                 s1*s2*c3 + c1*s3, -s1*s2*s3 + c1*c3, -s1*c2; ...
                 -c1*s2*c3 + s1*s3, c1*s2*s3 + s1*c3, c1*c2];
%             R = [c2*c3, -c1*s3 - s1*c2*c3, s1*s2; ...
%                  s1*c3 + c1*c2*s3, -s1*s3 + c1*c2*c3, -c1*s2; ...
%                  s2*s3, s2*c3, c2];
        end
    end
end



