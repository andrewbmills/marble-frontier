%% Create obstacle rich environment
% clear
close all
% cd('FastMarching_version3b');
% compile_c_files;
% cd('..');
width = 1;
height = 1;
minObsDist = 50; % it's just as fast to be at least 3 cells away as anywhere else.
% occGrid = drawOccGrid([m,n], 5);
image = imread('example_environment.PNG');
grayimage = rgb2gray(image);
occGrid = (grayimage/255)<=0.8;
% load('FrontierPaper40x40_Fig5.mat')
h1 = pcolor(occGrid);
set(h1, 'EdgeColor', 'none');
[m, n] = size(occGrid);
% axis([1 n+1 1 m+1]);
%% Generate robot(s)
x_agent = [144; 30; 0; 5]; % x (m), y (m), heading (rad from north), speed (m/s)
sensor = [50, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agent1 = agent(x_agent, agentGrid, [width, height], sensor);
agent1.sense(occGrid);
figure(3)
h2 = pcolor(agent1.occGrid);
set(h2, 'EdgeColor', 'none');
hold on
plot(agent1.state(1)+0.5, agent1.state(2)+0.5, 'r*')

%%
dt = 0.05;
while true    
    %% Create Reachability Grid
    speedGrid = bwdist(agent1.occGrid);
    satSpeedGrid = speedGrid;
    satSpeedGrid(satSpeedGrid >= minObsDist) = minObsDist;
    satSpeedGrid(satSpeedGrid == 0) = 1e-6;
    reachGrid = msfm(double(satSpeedGrid), [agent1.state(2); agent1.state(1)]);
%     reachGrid(reachGrid > 200) = Inf;

    %%  Find frontier grid cells (Open cells adjacent to unexplored cells)
    frontGrid = findFrontier(agent1.occGrid);
    if sum(frontGrid(:)) <= 10
        break
    end
    frontCost = frontGrid.*reachGrid + (1 - frontGrid)*1e6;
    [minCost, idNext] = min(frontCost(:));
    [i_goal, j_goal] = ind2sub(size(agent1.occGrid), idNext);
%     path = findPath(reachGrid, [i_goal, j_goal], [x_agent(2), x_agent(1)]);
    path = findPathContinuous(reachGrid, [i_goal, j_goal], [agent1.state(1), agent1.state(2)]);
    agent1.path = path;

    figure(3)
    h2 = pcolor(agent1.occGrid);
    hold on
    plot(agent1.state(1)+0.5,agent1.state(2)+0.5, 'r*')
    plot(agent1.path(:,1)+0.5, agent1.path(:,2)+0.5, 'r');
    plot(agent1.stateHistory(:,1)+0.5, agent1.stateHistory(:,2)+0.5, 'g-.');
    axis([1 n 1 m]);
    set(h2, 'EdgeColor', 'none');
    hold off
    
    %% Evolve state until the end of the path is no longer a frontier
%     isFrontier = 1;
    ii = 2;
    if size(path,2) == 1
        figure(3)
        h = pcolor(agent1.occGrid);
        hold on
        plot(agent1.state(1)+0.5,agent1.state(2)+0.5, 'r*')
        plot(agent1.path(:,1)+0.5, agent1.path(:,2)+0.5, 'r');
        plot(agent1.stateHistory(:,1)+0.5, agent1.stateHistory(:,2)+0.5, 'g-.');
        axis([1 n 1 m]);
        set(h, 'EdgeColor', 'none');
        hold off
        agent1.state(3) = wrapToPi(agent1.state(3) + pi/2);
        agent1.sense(occGrid); % sense environment
        continue
    end
    while frontierCheck(agent1.path(end,2), agent1.path(end,1), agent1.occGrid)
        figure(3)
        h = pcolor(agent1.occGrid);
        hold on
        plot(agent1.state(1)+0.5,agent1.state(2)+0.5, 'r*')
        plot(agent1.path(:,1)+0.5, agent1.path(:,2)+0.5, 'r');
        plot(agent1.stateHistory(:,1)+0.5, agent1.stateHistory(:,2)+0.5, 'g-.');
        axis([1 n 1 m]);
        set(h, 'EdgeColor', 'none');
        hold off
        agent1.move(dt, 5);
        agent1.sense(occGrid); % sense environment
        pause(dt);
    end
    
    % Check for collision
    if occGrid(floor(agent1.state(2)), floor(agent1.state(1)))
        error('vehicle collided with environment');
    end
end