% Modifed from https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/visualize.m
function visualize(sys, t, q, u)

    hFig = figure(25); set(hFig,'DoubleBuffer', 'on');
    figure(25); 
    plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
        
    % Initialize figures
    subplot(plots(1)); grid on;
%     cla; hold on; view(3); 
    view(3); 
    
    data1 = q(sys.indices.x, :);
    subplot(plots(2)); hold on;
    xlabel('Time (s)'); ylabel('Value (m)');        
    title('Position');        
    xmin = min(t); xmax = max(t); ymin = 1.1 * min(min(data1)); ymax = 1.1 * max(max(data1));
    axis([xmin xmax ymin ymax]);     
    
    data2 = q(sys.indices.varphi, :);    
    subplot(plots(3)); hold on;
    xlabel('Time (s)'); ylabel('Angles (rad)');
    xmin = min(t); xmax = max(t); ymin = 1.1 * min(min(data2)); ymax = 1.1 * max(max(data2));
    axis([xmin xmax ymin ymax]);    
  
    % Go through time stamps
    for i = 1:size(t,1)

        %% Quadrotor figure
        subplot(plots(1)); cla;     
        [r_fig, T_fig] = quadVis(); % quad and thrust draw
        [x, v, varphi, omega] = sys.getStateMember(q(:,i));
        R = sys.getRotMat(varphi);
        B01 = [R, x; 0, 0, 0, 1];   % homogeneous transformation matrix
        set(r_fig, 'Matrix', B01);  % place the quadrotor

        % The cylinders for thrusts
        % Compute scaling for the thrust cylinders, based on heuristic scaling
    %     scales = exp(data.input(:, t) / min(abs(data.input(:, t))) + 5) - exp(6) +  1.5;
        scales = 5* ones(4,1);
        for j = 1:4
            % For negative scales, we need to flip the cylinder
            s = scales(j);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end
            % Place the thrusts cylinders
            set(T_fig(j), 'Matrix', B01 * scalez);
        end

        % Update axis
        mg = [20, 20, 5]; % margin from quadrotor to the limit of axis of drawing
        axis image;
        axis([x(1)-mg(1), x(1)+mg(1), x(2)-mg(2), x(2)+mg(2), x(3)-mg(3), x(3)+mg(3)]);

        zlabel('z'); xlabel('x'); ylabel('y');
        title(['t = ', num2str(t(i),'%.2f') ' sec']);
    %     set(gca,'XTick',[],'YTick',[])
        drawnow;

        %% Other two figures, plot whatever you want
        ts = t(1:i);
        subplot(plots(2)); cla;
        plot(ts, data1(1, 1:i), 'r',  ts, data1(2, 1:i), 'g', ts, data1(3, 1:i), 'b', 'LineWidth', 2);
        plot(t(i), data1(1,i), 'or', t(i), data1(2,i), 'og', t(i), data1(3,i), 'ob', 'MarkerSize',10);
%         legend('x', 'y', 'z');        
        
        subplot(plots(3)); cla;
        plot(ts, data2(1, 1:i), 'r', ts, data2(2, 1:i), 'g', ts, data2(3, 1:i), 'b', 'LineWidth', 2);        
        plot(t(i), data2(1,i), 'or', t(i), data2(2,i), 'og', t(i), data2(3,i), 'ob', 'MarkerSize',10);        
%         legend('row', 'pitch', 'yaw'); title('Euler Angles');        


    end
end



% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders.
% These will be transformed during the animation to display
% relative thrust forces.
function [h, thrusts] = quadVis()
    % Draw arms.
    h(1) = prism(-5, -0.25, -0.25, 10, 0.5, 0.5);
    h(2) = prism(-0.25, -5, -0.25, 0.5, 10, 0.5);

    % Draw bulbs representing propellers at the end of each arm.
    [x, y, z] = sphere();
    x = 0.5 * x;
    y = 0.5 * y;
    z = 0.5 * z;
    h(3) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x, y, z] = cylinder(0.1, 7);
    thrusts(1) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(2) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');
    thrusts(3) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(4) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end


% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism( x, y, z, w, l, h)
    [X Y Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X Y Z] = prism_faces( x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end


