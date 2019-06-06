classdef Agent < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        type
        state
        stateHistory
        path
        goalType
        cost = 0 % Cost to reach the current goal point
        tLook % Lookahead time for guidance controller
%         carrot % path distance parameter
        occGrid
        gridDims
        sensor % [range (m), field of view (radians]
        controlLims % [maxSpeed, maxTurnRate, maxDecel]
        neighbors = Neighbor.empty % array of the agents current in communication with self
        artifacts = Artifact.empty % array of artifacts seen
        beacons = 0 % number of beacons the agent is carrying
        run
    end
   
    methods
        function obj = Agent(id, type, state0, occGrid0, gridDims, sensor, numBeacons, maxControl)
            obj.id = id;
            obj.type = type;
            obj.state = state0; % [x_pos, y_pos, angle, speed]
            obj.stateHistory = [0, obj.state'];
            obj.occGrid = occGrid0;
            obj.gridDims = gridDims;
            obj.tLook = 1; % seconds
            obj.sensor = [10; 2*pi];
            obj.controlLims = [5; 1800*pi/180; 0.1*9.81];
            obj.path = [state0(1), state0(2)];
            if strcmp(type, 'beacon')
                obj.run = false;
            else
                obj.run = true;
            end
%             obj.carrot = [0, 1]; % 0 distance along path segment 1
            obj.sensor = sensor;
            obj.beacons = numBeacons;
            if nargin == 8
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
            t0 = obj.stateHistory(end, 1);
            
            % Save state history
            obj.stateHistory = [obj.stateHistory; t(2:end) + t0, x(2:end, :)];
%             if size(obj.stateHistory, 1) >= 10000
%                 obj.stateHistory = obj.stateHistory((end-10000):end, :);
%             end
        end
        
        function sense(obj, trueGrid)
            obj.occGrid = senseGrid(obj.state, trueGrid, obj.occGrid, ...
                obj.gridDims, obj.sensor);
        end
        
        function detect(obj, artifacts)
            obj.artifacts = detectGrid(obj.state, artifacts, obj.artifacts, obj.occGrid, ...
                obj.gridDims, obj.sensor);
        end

        function plot(obj)
            h = pcolor(obj.occGrid);
            hold on
            plot(obj.state(1)+0.5, obj.state(2)+0.5, 'r*')
            if ~isempty(obj.path)
                plot(obj.path(:,1)+0.5, obj.path(:,2)+0.5, 'r');
            end
            plot(obj.stateHistory(:,2)+0.5, obj.stateHistory(:,3)+0.5, 'g-.');

            % Plot the detected artifacts
            for artifact = obj.artifacts
                plot(artifact.pos(1), artifact.pos(2), 'w+');
                text(artifact.pos(1)+1.5, artifact.pos(2)+1.5, artifact.type, 'Color', 'white', 'FontSize', 12);
            end

            [m, n] = size(obj.occGrid);
            axis([1 n 1 m]);
            set(h, 'EdgeColor', 'none');
            axis equal
            axis tight
            hold off
        end
        
        function [goal, idx] = deconflictGoal(obj, frontCost, idNext)
            %% Deconflict the goal point with all of the neighbor agents
            global deconfliction;
            global nogoalBehavior;
            global sensorRange;
            i = 1;
            % If there are no neighbors, we can just go to the lowest cost frontier
            if isempty(obj.neighbors)
                [y_test, x_test] = ind2sub(size(obj.occGrid), idNext(1));
                goal = [x_test, y_test];
                idx = 1;
                obj.cost = frontCost(idNext(1));
                obj.goalType = 'explore';
            else
                conflict = true;

                % Search through each of the low cost frontiers in order
                % If we reach one with a cost of 1000000 it's outside the map
                while conflict && i <= length(idNext) && frontCost(idNext(i)) < 1000000
                    % Get the goal point
                    [y_test, x_test] = ind2sub(size(obj.occGrid), idNext(i));
                    goal = [x_test, y_test];
                    idx = i;
                    obj.cost = frontCost(idNext(i));
                    obj.goalType = 'explore';
                    % Check the distance between our goal and each neighbors' goal
                    for neighbor = obj.neighbors
                        if ~isempty(neighbor.goal) && strcmp(neighbor.goalType, 'explore')
                            dist = sqrt(sum((goal - neighbor.goal) .^ 2));
                            % If it's close, we don't need to look at anymore
                            % neighbors, we just need to look at our next goal
                            % TODO, check who has the lower cost to do this goal
                            % TODO, figure out best distance, maybe sensor width?
                            if dist < sensorRange
                                % See who has the lower cost.  If it's self, take the path and flag the neighbor to change his
                                % Otherwise, skip the other neighbors (since they can't have the same goal point due to previous
                                % iterations) and look at the next goal point
                                if deconfliction > 1 && obj.cost < neighbor.cost
                                    conflict = false;
                                    neighbor.replan = true;
                                else
                                    conflict = true;
                                end
                                break;
                            end
                        end
                        % If we get through all of the neighbors without being too
                        % close, there's no conflict, and this will remain false,
                        % ending the outer loop
                        conflict = false;
                    end

                    i = i + 1;
                end
            end

            % If we ran out of possible frontiers, set the frontier to the current position,
            % so that we keep checking every cycle until there is no more frontier
            if i - 1 == length(idNext) || frontCost(idNext(i)) == 1000000
                if nogoalBehavior == 1
                    % Continue on to our first goal point
                    [y_test, x_test] = ind2sub(size(obj.occGrid), idNext(1));
                    goal = [x_test, y_test];
                    idx = 1;
                    obj.cost = frontCost(idNext(1));
                    obj.goalType = 'follow';
                elseif nogoalBehavior == 2
                    % Follow our nearest neighbor
                    goal = [];
                    obj.cost = 0;
                    obj.goalType = 'follow';
                    closest = 10000;
                    for neighbor = obj.neighbors
                        % Check neighbor is the closer than any others we've looked at,
                        % is at least sensorRange ahead, and has a path other than current position
                        dist = sqrt(sum((obj.state(1:2) - neighbor.pos(1:2)) .^ 2));
                        if dist < closest && dist > sensorRange && length(neighbor.path) > 2
                            goal = [round(neighbor.pos(1)), round(neighbor.pos(2))];
                            idx = 0;
                            closest = dist;
                        end
                    end
                else
                    goal = [];
                    idx = 0;
                    obj.cost = 0;
                    obj.goalType = 'wait';
                end
            end
        end

        function frontierPlan(obj, anchorGoal, hblob, minObsDist, figNum)
            %   (anchor position, blob detector, minimum obstacle distance, figure number)
            %% Create Reachability Grid
            speedGrid = bwdist(obj.occGrid);
            satSpeedGrid = speedGrid;
            if nargin == 3
                satSpeedGrid(satSpeedGrid >= minObsDist) = minObsDist;
            end
            satSpeedGrid(satSpeedGrid == 0) = 1e-6;
            reachGrid = msfm(double(satSpeedGrid), [obj.state(2); obj.state(1)]);

            %%  Find frontier grid cells (Open cells adjacent to unexplored cells)
            frontGrid = findFrontier(obj.occGrid);
            if nargin >= 4
                [~, centroids, bbox, labels] = step(hblob, logical(frontGrid));
            end
            frontGrid = double(labels >= 1);
            if sum(frontGrid(:)) <= 10
                obj.path = [];
                obj.cost = 0;
                return
            end

            % Find the cost to reach the frontiers
            frontCost = frontGrid.*reachGrid + (1 - frontGrid)*1e6;
            % If there's an anchor goal we need the cost to reach it
            frontCost(anchorGoal) = reachGrid(anchorGoal);
            % Find the N lowest cost frontiers
            % TODO decide on optimum N.  >150 was seen once.
            % This also needs to be changed in a couple other places if changed here
            [~, idNext] = mink(frontCost(:), 500);
            % Add the anchor node as our primary goal if that's what was passed
            idNext = [anchorGoal; idNext];

            % Check all of the neighbors to decide which goal to explore
            [goal, idx] = obj.deconflictGoal(frontCost, idNext);

            % If there's no goal (potentially because all of them are already taken by other agents), stay in place for now
            if isempty(goal)
                obj.path = [round(obj.state(1)), round(obj.state(2))];
                return
            end

            % Compute the path to the chosen frontier
            obj.path = findPathContinuous(reachGrid, goal, [obj.state(1), obj.state(2)]);

            % TODO this is all a hack!
            % If there's no path, try the next goal point
            if obj.path == 0
                idNext = idNext(idx+1:end);
                [goal, ~] = obj.deconflictGoal(frontCost, idNext);
                if isempty(goal)
                    obj.path = [round(obj.state(1)), round(obj.state(2))];
                    return
                end

                obj.path = findPathContinuous(reachGrid, goal, [obj.state(1), obj.state(2)]);

                % If there's still not a path, teleport backwards
                if obj.path == 0
                    obj.state = obj.stateHistory(end-200,2:end)';
                    obj.path = findPathContinuous(reachGrid, goal, [obj.state(1), obj.state(2)]);
                end
            end

            global gridPlots;
            if nargin == 4 && gridPlots
                figure(figNum)
                subplot(2,2,1);
                h = pcolor(satSpeedGrid);
                set(h, 'EdgeColor', 'none');
                title('Saturated ESDF')
                ax2 = subplot(2,2,2);
                maxReachFree = 50;
                reachGrid(reachGrid >= maxReachFree) = maxReachFree;
                %         h = pcolor(reachGrid);
                %         hold on
                h = contourf(log(reachGrid), 'LevelStep', 0.5);
                colormap(ax2, cool)
                %         set(h, 'EdgeColor', 'none');
                title('Reachability (Cost)')
                subplot(2,2,3)
                h = pcolor(frontGrid);
                set(h, 'EdgeColor', 'none');
                hold on
                for i = 1:size(bbox,1)
                    plot(centroids(i,1), centroids(i,2));
                    rectangle('Position', bbox(i,:));
                end
                hold off
                title('Frontier')
                subplot(2,2,4)
                h = pcolor(obj.occGrid);
                set(h, 'EdgeColor', 'none');
                hold on
                plot(obj.path(:,1), obj.path(:,2), 'r');
                plot(obj.state(1), obj.state(2), 'r*');
                title('Path')
                hold off
                set(gcf, 'Position', [1, 1, 1080, 1080]);
                axis equal
                axis tight
                tightfig;
            end
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
        ind = size(path,1)-1;
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