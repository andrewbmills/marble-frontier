%% Create obstacle rich environment
clear
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

%% Some global variables to define behavior
global gridPlots;
gridPlots = 0; % Plot the frontier plots or not
global plotFused;
plotFused = 1; % Plot all robots on one graph
global plotAgents;
plotAgents = 1; % Plot individual robot views on single graph, automatically sized

%% Use this to decide how many agents to run
numAgents = 4;

%% Generate robot(s)
x_agent = [144; 30; 0; 5]; % x (m), y (m), heading (rad from north), speed (m/s)
% x_agent = [125; 125; 0; 5];
sensor = [50, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agents(1,1) = Agent(x_agent, agentGrid, [width, height], sensor);
agents(1,1).sense(occGrid);

x_agent = [172; 240; pi; 5];
sensor = [50, 2*pi];
agents(1,2) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
agents(1,2).sense(occGrid);

if numAgents > 2
    x_agent = [100; 200; pi; 5];
    sensor = [50, 2*pi];
    agents(1,3) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
    agents(1,3).sense(occGrid);
end

if numAgents > 3
    x_agent = [120; 50; 270*pi/180; 5];
    sensor = [50, 2*pi];
    agents(1,4) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
    agents(1,4).sense(occGrid);
end

if numAgents > 4
    x_agent = [140; 20; 0; 5];
    sensor = [50, 2*pi];
    agents(1,5) = Agent(x_agent, agentGrid, [width, height], sensor);
    agents(1,5).sense(occGrid);
end

if numAgents > 5
    x_agent = [160; 230; pi; 5];
    sensor = [50, 2*pi];
    agents(1,6) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
    agents(1,6).sense(occGrid);
end

if numAgents > 6
    x_agent = [110; 210; pi; 5];
    sensor = [50, 2*pi];
    agents(1,7) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
    agents(1,7).sense(occGrid);
end

if numAgents > 7
    x_agent = [110; 70; 270*pi/180; 5];
    sensor = [50, 2*pi];
    agents(1,8) = Agent(x_agent, 0.5*ones(m,n), [width, height], sensor);
    agents(1,8).sense(occGrid);
end

if numAgents > 8
    x_agent = [144; 30; 0; 5];
    sensor = [50, 2*pi];
    agents(1,9) = Agent(x_agent, agentGrid, [width, height], sensor);
    agents(1,9).sense(occGrid);
end

if plotFused
    plotFusedGrid(agents, 2);
end
if plotAgents
    plotAgentGrid(agents, 3);
end
    
%% Blob detector
hblob = vision.BlobAnalysis;
hblob.LabelMatrixOutputPort = true;
hblob.MinimumBlobArea = 10;

%% Simulate
t = 0;
dt = 0.5; % physics time step
dt_plan = 1.0; % re-planning time step
dt_plot = 0.5; % plotting time step
gif_start = ones(4,1);
filenames = {'robot1OccMap.gif', 'robot2OccMap.gif', 'robot1planner.gif', 'robot2planner.gif'};
% filenames = {'robotAloneOccMap.gif', 'robotAlonePlanner.gif'};
gif_flag = 0; % Make .gif files boolean

while any([agents.run])
    % Plan Path(s)
    if (mod(t, dt_plan) == 0)
        for agent = agents
            if ~isempty(agent.path)
                if ~frontierCheck(agent.path(end,2), agent.path(end,1), agent.occGrid)
                    agent.path = frontierPlan(agent.occGrid, agent.state(1:2), hblob, minObsDist, 5); % (occupancy grid, agent position, blob detector, minimum obstacle distance, figure number)
                end
            else
                agent.run = false;
            end
        end
    end
    
    % Plot vehicle positions and belief grid
    if (mod(t, dt_plot) == 0)
        if plotFused
            plotFusedGrid(agents, 2);
        end
        if plotAgents
            plotAgentGrid(agents, 3);
        end
    end
    
    for agent = agents
        % Evolve state until the end of the path is no longer a frontier
        if size(agent.path,2) == 1
            agent.state(3) = wrapToPi(agent.state(3) + pi/2);
        else
            if ~isempty(agent.path)
                agent.move(dt, 5);
            end
        end
    
        % Sense
        agent.sense(occGrid);
    
        % Check for collision
        if occGrid(floor(agent.state(2)), floor(agent.state(1)))
            % TODO fix this to display correct vehicle
            error('vehicle 1 collided with environment');
        end
    end
    
    % Comm
    for i=1 : length(agents)
        for j=i+1 : length(agents)
            if checkLoS(agents(i).state(1:2), agents(j).state(1:2), occGrid)
                fusedGrid = fuseOccGrids(agents(i).occGrid, agents(j).occGrid);
                agents(i).occGrid = fusedGrid;
                agents(j).occGrid = fusedGrid;
            end
        end
    end
    
    % Add frames to .gif file
    % TODO fix this for new >2 vehicle code
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
    
    t = t + dt;
%     pause(dt);
end

disp(t)

if plotFused
    plotFusedGrid(agents, 2);
end
if plotAgents
    plotAgentGrid(agents, 3);
end
drawnow

% TODO fix this for new >2 vehicle code
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