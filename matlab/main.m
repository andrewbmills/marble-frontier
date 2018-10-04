%% Create obstacle rich environment
% clear
close all
cd('FastMarching_version3b');
compile_c_files;
cd('..');
width = 1;
height = 1;
minObsDist = 15; % it's just as fast to be at least x cells away as anywhere else.
% occGrid = drawOccGrid([m,n], 5);
image = imread('example_environment.PNG');
grayimage = rgb2gray(image);
occGrid = (grayimage/255)<=0.8;
% load('FrontierPaper40x40_Fig5.mat')
h1 = pcolor(occGrid);
set(h1, 'EdgeColor', 'none');
[m, n] = size(occGrid);

%% Generate robot(s)
x_agent = [144; 30; 0; 5]; % x (m), y (m), heading (rad from north), speed (m/s)
% x_agent = [125; 125; 0; 5];
sensor = [50, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agent1 = agent(x_agent, agentGrid, [width, height], sensor);
agent1.sense(occGrid);
agent1.plot(3)

x_agent = [172; 240; pi; 5];
sensor = [50, 2*pi];
agent2 = agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
agent2.sense(occGrid);
agent2.plot(4)

%% Blob detector
hblob = vision.BlobAnalysis;
hblob.LabelMatrixOutputPort = true;
hblob.MinimumBlobArea = 10;

%% Simulate
dt = 0.5;
gif_start = ones(4,1);
filenames = {'robot1OccMap.gif', 'robot2OccMap.gif', 'robot1planner.gif', 'robot2planner.gif'};
% filenames = {'robotAloneOccMap.gif', 'robotAlonePlanner.gif'};
gif_flag = 1;
while true
    % Plan Path(s)
    if ~frontierCheck(agent1.path(end,2), agent1.path(end,1), agent1.occGrid)
        agent1.path = frontierPlan(agent1.occGrid, agent1.state(1:2), hblob, minObsDist, 5);
        if isempty(agent1.path)
            break
        end
    end
    if ~frontierCheck(agent2.path(end,2), agent2.path(end,1), agent2.occGrid)
        agent2.path = frontierPlan(agent2.occGrid, agent2.state(1:2), hblob, minObsDist, 6);
        if isempty(agent2.path)
            break
        end
    end
    agent1.plot(3)
    agent2.plot(4)
    
    % Evolve state until the end of the path is no longer a frontier
    if size(agent1.path,2) == 1
        agent1.state(3) = wrapToPi(agent1.state(3) + pi/2);
    else
        agent1.move(dt, 5);
    end
    if size(agent2.path,2) == 1 
        agent2.state(3) = wrapToPi(agent2.state(3) + pi/2);
    else
        agent2.move(dt, 5);
    end
    
    % Sense
    agent1.sense(occGrid);
    agent2.sense(occGrid);
    
    % Comm
    if checkLoS(agent1.state(1:2), agent2.state(1:2), occGrid)
        agentGrid = fuseOccGrids(agent1.occGrid, agent2.occGrid);
        agent1.occGrid = agentGrid;
        agent2.occGrid = agentGrid;
    end
    
    % Check for collision
    if occGrid(floor(agent1.state(2)), floor(agent1.state(1)))
        error('vehicle 1 collided with environment');
    end
    if occGrid(floor(agent2.state(2)), floor(agent2.state(1)))
        error('vehicle 2 collided with environment');
    end
    
    % Add frames to .gif file
    drawnow
    if gif_flag
        for fig_num = 3:6
            % Add frame to gif
            frame = getframe(fig_num);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if gif_start(fig_num-2)
                imwrite(imind,cm,filenames{fig_num-2},'gif', 'Loopcount',0);
                gif_start(fig_num-2) = 0;
            else
                imwrite(imind,cm,filenames{fig_num-2},'gif','WriteMode','append','DelayTime',0);
            end
        end
    end
    
%     pause(dt);
end

agent1.plot(3)
agent2.plot(4)
drawnow
if gif_flag
    for fig_num = 3:4
        % Add frame to gif
        frame = getframe(fig_num);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if gif_start(fig_num-2)
            imwrite(imind,cm,filenames{fig_num-2},'gif', 'Loopcount',0);
            gif_start(fig_num-2) = 0;
        else
            imwrite(imind,cm,filenames{fig_num-2},'gif','WriteMode','append','DelayTime',5);
        end
    end
end