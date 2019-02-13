function [path] = goalPlan(occGrid, position, goal, minObsDist, figNum)
%   (occupancy grid, agent position, agent goal, minimum obstacle distance, figure number)
    %% Create Reachability Grid
    speedGrid = bwdist(occGrid);
    satSpeedGrid = speedGrid;
    if nargin == 4
        satSpeedGrid(satSpeedGrid >= minObsDist) = minObsDist;
    end
    satSpeedGrid(satSpeedGrid == 0) = 1e-6;
    reachGrid = msfm(double(satSpeedGrid), [position(2); position(1)]);

    %%  Find frontier grid cells (Open cells adjacent to unexplored cells)
    path = findPathContinuous(reachGrid, [goal(2), goal(1)], [position(1), position(2)]);
    
    if nargin == 5
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
        h = pcolor(occGrid);
        set(h, 'EdgeColor', 'none');
        hold on
        plot(goal(1), goal(2), 'g*');
        hold off
        title('Goal Point')
        subplot(2,2,4)
        h = pcolor(occGrid);
        set(h, 'EdgeColor', 'none');
        hold on
        plot(path(:,1), path(:,2), 'r');
        plot(position(1), position(2), 'r*');
        title('Path')
        hold off
        set(gcf, 'Position', [1, 1, 1080, 1080]);
        axis equal
        axis tight
        tightfig;
    end
    
end