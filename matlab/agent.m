classdef agent < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        state
        stateHistory
        path
        tLook % Lookahead time for guidance controller
%         carrot % path distance parameter
        occGrid
        gridDims
        sensor
        controlLims % [maxSpeed, maxTurnRate, maxDecel]
    end
   
    methods
        function obj = agent(state0, occGrid0, gridDims, sensor, maxControl)
            obj.state = state0; % [x_pos, y_pos, angle, speed]
            obj.stateHistory = obj.state';
            obj.occGrid = occGrid0;
            obj.gridDims = gridDims;
            obj.tLook = 1; % seconds
            obj.sensor = [10; 2*pi];
            obj.controlLims = [5; 180*pi/180; 0.1*9.81];
%             obj.carrot = [0, 1]; % 0 distance along path segment 1
            if nargin == 4
                obj.sensor = sensor;
            elseif nargin == 5
                obj.controlLims = maxControl;
            end
        end
        
        function move(obj, dt, V_command)
            % Move the carrot
%             i_path= obj.carrot(2);
%             if i_path >= size(obj.path,1)-1
%                 p1 = obj.path(end-1, 1:2);
%                 p2 = obj.path(end, 1:2);
%                 obj.carrot(1) = min(dt*obj.path(i_path, 3)/norm(p2-p1), 1);
%             else
%                 p1 = obj.path(i_path, 1:2);
%                 p2 = obj.path(min(i_path + 1, size(obj.path,1)), 1:2);
%                 obj.carrot(1) = dt*obj.path(i_path, 3)/norm(p2-p1);
%             end
%             while obj.carrot(1) > 1
%                 obj.carrot(2) = obj.carrot(2) + 1;
%                 i_path = obj.carrot(2);
%                 if i_path >= size(obj.path,1)
%                     obj.carrot(1) = 1;
%                     break
%                 end
%                 p0 = obj.path(i_path-1, 1:2);
%                 p1 = obj.path(i_path, 1:2);
%                 p2 = obj.path(min(i_path + 1, size(obj.path,1)), 1:2);
%                 obj.carrot(1) = (obj.carrot(1) - 1)*norm(p2-p1)/norm(p1-p0);
%             end
            
            % Move the agent
            R = obj.state(4)*obj.tLook;
            
            [t, x] = ode45(@(t,x) unicycleODE(t, x, R, obj.path, ...
                obj.controlLims, V_command), [0 dt], obj.state);
            obj.state = x(end,:)';
            
            % Save state history
            obj.stateHistory = [obj.stateHistory; x(2:end, :)];
            if size(obj.stateHistory, 1) >= 1000
                obj.stateHistory = obj.stateHistory((end-1000):end, :);
            end
        end
        
        function sense(obj, trueGrid)
            obj.occGrid = senseGrid(obj.state, trueGrid, obj.occGrid, ...
                obj.gridDims, obj.sensor);
        end
    end
end

function d = angle_diff(a, b)
    % Computes a-b, preserving the correct sign (CCW positive angles).
    % Angles are in degrees.
    a = mod(360000 + a, 360);
    b = mod(360000 + b, 360);
    d = a - b;
    d = mod(d + 180, 360) - 180;
end

function psi_dot = trajectoryShapingGuidance(p_L2, p_AC, v_AC, v_path)
    Vg = norm(v_AC); % ground speed (m/s)
    L2 = p_L2 - p_AC; % Vector from current position to lookahead point
    t_go = norm(L2)/Vg; % Time until lookahead point is reached at Vg
    
    % Angles for trajectory shaping
    theta = atan2(L2(2), L2(1))*180/pi;
    alpha_a = atan2(v_AC(2), v_AC(1))*180/pi;
    alpha_t = atan2(v_path(2), v_path(1))*180/pi;
    
    % Calculate ccommanded angle rate
    psi_dot = (4*angle_diff(alpha_a, theta) + 2*angle_diff(alpha_t, theta))*...
        pi/(180*t_go);
end

function [L, vhat] = findLookahead(p_AC, R, path)
    N = size(path,1);
    d = path(2:end, :) - path(1:end-1, :); % N-1 x 2
    q = path(1:end-1, :) - repmat(p_AC, N-1, 1);
    a = sum(d.^2, 2);
    b = 2*sum(d.*q, 2);
    c = sum(q.^2, 2) - R^2;
    discrim = b.^2 - 4*a.*c;
    tHat = -1*ones(size(discrim));
    tHat1 = (-b + sqrt(discrim)) ./ (2*a);
    tHat2 = (-b - sqrt(discrim)) ./ (2*a);
    tHat(discrim >= 0) = max(tHat1(discrim >= 0), tHat2(discrim >= 0));
    intersection = (tHat >= 0).*(tHat <=1);
    ind = find(intersection, 1, 'last');
    if isempty(ind)
        [~, ind] = min(norm(q, 2));
        L = path(ind, :);
    else
        L = d(ind, :)*tHat(ind) + path(ind, :);
    end
    vhat = d(ind, :)/norm(d(ind, :));
end

function [xdot] = unicycleODE(t, x, R, path, controlLims, V_c)
    [L, vhat] = findLookahead(x(1:2)', R, path);
    v = [x(4)*sin(x(3)), x(4)*cos(x(3))];
    psi_dot = trajectoryShapingGuidance(L, x(1:2)', v, vhat);
    xdot = zeros(4,1);
    xdot(1) = v(1);
    xdot(2) = v(2);
    xdot(3) = min(psi_dot, controlLims(2)); % saturate the commanded turn rate
    if V_c >= controlLims(1)
        V_c = controlLims(1);
    end
    xdot(4) = min(0.1*(V_c - x(4)), controlLims(3)); % change this later
end