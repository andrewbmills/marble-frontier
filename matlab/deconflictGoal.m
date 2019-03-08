function [goal, cost] = deconflictGoal(occGrid, frontCost, idNext, neighbors)
    %% Deconflict the goal point with all of the neighbor agents
    i = 1;
    % If there are no neighbors, we can just go to the lowest cost frontier
    if isempty(neighbors)
        [y_test, x_test] = ind2sub(size(occGrid), idNext(1));
        goal = [x_test, y_test];
        cost = frontCost(idNext(1));
    else
        conflict = true;

        % Search through each of the low cost frontiers in order
        % If we reach one with a cost of 1000000 it's outside the map
        while conflict && i <= length(idNext) && frontCost(idNext(i)) < 1000000
            % Get the goal point
            [y_test, x_test] = ind2sub(size(occGrid), idNext(i));
            goal = [x_test, y_test];
            cost = frontCost(idNext(i));
            % Check the distance between our goal and each neighbors' goal
            for neighbor = neighbors
                if ~isempty(neighbor.goal)
                    dist = sqrt(sum((goal - neighbor.goal) .^ 2));
                    % If it's close, we don't need to look at anymore
                    % neighbors, we just need to look at our next goal
                    % TODO, check who has the lower cost to do this goal
                    % TODO, figure out best distance, maybe sensor width?
                    if dist < 50
                        if neighbor.id == 1
                            test = 1;
                        end
                        % See who has the lower cost.  If it's self, take the path and flag the neighbor to change his
                        % Otherwise, skip the other neighbors (since they can't have the same goal point due to previous
                        % iterations) and look at the next goal point
                        if cost < neighbor.cost
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

    % If we ran out of possible frontiers, set the frontier to the current
    % position, so that we keep checking every cycle until there is no more
    % frontier
    % TODO instead of stopping, change the role from explore to search or
    % comm relay
    if i - 1 == 500 || frontCost(idNext(i)) == 1000000
        goal = [];
        cost = [];
    end

end

