%% Create obstacle rich environment
% clear
close all
cd('FastMarching_version3b');
compile_c_files;
cd('..');
m = 40;
n = 40;
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
x_agent = [176; 238; 0; 10]; % x (m), y (m), heading (rad from north), speed (m/s)
u_agent = [0; 0]; % heading rate (rad/s), acceleration (m/s^2)
sensor = [50, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agentGrid = senseGrid(x_agent, occGrid, agentGrid, [width, height], sensor);
figure(3)
h2 = pcolor(agentGrid);
set(h2, 'EdgeColor', 'none');
hold on
plot(x_agent(1)+0.5, x_agent(2)+0.5, 'r*')

%% Plot Speed and Reachability Grids
% speedGrid = bwdist(agentGrid);
% figure
% pcolor(speedGrid);
% satSpeedGrid = speedGrid;
% satSpeedGrid(satSpeedGrid >= minObsDist) = minObsDist;
% figure
% pcolor(satSpeedGrid);
% 
% satSpeedGrid(satSpeedGrid == 0) = 1e-4;
% reachGrid = msfm(double(satSpeedGrid), x_agent(1:2));
% reachGrid(reachGrid > 200) = Inf;
% figure
% pcolor(reachGrid);

%%
while true    
    %% Create Reachability Grid
    speedGrid = bwdist(agentGrid);
    satSpeedGrid = speedGrid;
    satSpeedGrid(satSpeedGrid >= minObsDist) = minObsDist;
    satSpeedGrid(satSpeedGrid == 0) = 1e-6;
    reachGrid = msfm(double(satSpeedGrid), [x_agent(2); x_agent(1)]);
%     reachGrid(reachGrid > 200) = Inf;

    %%  Find frontier grid cells (Open cells adjacent to unexplored cells)
    frontGrid = findFrontier(agentGrid);
    if sum(frontGrid(:)) <= 10
        break
    end
    frontCost = frontGrid.*reachGrid + (1 - frontGrid)*1e6;
    [minCost, idNext] = min(frontCost(:));
    [i_goal, j_goal] = ind2sub(size(agentGrid), idNext);
    path = findPath(reachGrid, [i_goal, j_goal], [x_agent(2), x_agent(1)]);
%     path = findPathContinuous(reachGrid, [i_goal, j_goal], [x_agent(2), x_agent(1)]);

    figure(3)
    h2 = pcolor(agentGrid);
    hold on
    plot(x_agent(1)+0.5, x_agent(2)+0.5, 'r*')
    plot(path(:,1)+0.5, path(:,2)+0.5, 'r');
    axis([1 n 1 m]);
    set(h2, 'EdgeColor', 'none');
    hold off
    
    %% Evolve state until the end of the path is no longer a frontier
%     isFrontier = 1;
    ii = 2;
    if size(path,2) == 1
        figure(3)
        h = pcolor(agentGrid);
        hold on
        plot(x_agent(1)+0.5, x_agent(2)+0.5, 'r*')
        plot(path(:,1)+0.5, path(:,2)+0.5, 'r')
        axis([1 n 1 m]);
        set(h, 'EdgeColor', 'none');
        hold off
        x_agent(3) = wrapToPi(x_agent(3) + pi/2);
        agentGrid = senseGrid(x_agent, occGrid, agentGrid, [width, height], sensor); % sense environment
        continue
    end
    for ii = 2:x_agent(4):size(path,2)
        figure(3)
        h = pcolor(agentGrid);
        hold on
        plot(x_agent(1)+0.5, x_agent(2)+0.5, 'r*')
        plot(path(:,1)+0.5, path(:,2)+0.5, 'r')
        axis([1 n 1 m]);
        set(h, 'EdgeColor', 'none');
        hold off
        x_agent(3) = atan2(path(1, ii) - x_agent(1), path(2,ii) - x_agent(2));
        x_agent(1) = path(ii, 1); % move along path
        x_agent(2) = path(ii, 2);
        agentGrid = senseGrid(x_agent, occGrid, agentGrid, [width, height], sensor); % sense environment
        % Repetitive Re-Checking
        if ~frontierCheck(path(2,end), path(1,end), agentGrid)
            break
        end
    end
end