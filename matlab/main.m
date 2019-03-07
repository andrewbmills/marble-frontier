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
agents(1,1) = Agent(1, x_agent, agentGrid, [width, height], sensor);
agents(1,1).sense(occGrid);

x_agent = [172; 240; pi; 5];
sensor = [50, 2*pi];
agents(1,2) = Agent(2, x_agent, agentGrid, [width, height], sensor);
agents(1,2).sense(occGrid);

if numAgents > 2
    x_agent = [100; 200; pi; 5];
    sensor = [50, 2*pi];
    agents(1,3) = Agent(3, x_agent, agentGrid, [width, height], sensor);
    agents(1,3).sense(occGrid);
end

if numAgents > 3
    x_agent = [120; 50; 270*pi/180; 5];
    sensor = [50, 2*pi];
    agents(1,4) = Agent(4, x_agent, agentGrid, [width, height], sensor);
    agents(1,4).sense(occGrid);
end

if numAgents > 4
    x_agent = [140; 20; 0; 5];
    sensor = [50, 2*pi];
    agents(1,5) = Agent(5, x_agent, agentGrid, [width, height], sensor);
    agents(1,5).sense(occGrid);
end

if numAgents > 5
    x_agent = [160; 230; pi; 5];
    sensor = [50, 2*pi];
    agents(1,6) = Agent(6, x_agent, agentGrid, [width, height], sensor);
    agents(1,6).sense(occGrid);
end

if numAgents > 6
    x_agent = [110; 210; pi; 5];
    sensor = [50, 2*pi];
    agents(1,7) = Agent(7, x_agent, agentGrid, [width, height], sensor);
    agents(1,7).sense(occGrid);
end

if numAgents > 7
    x_agent = [110; 70; 270*pi/180; 5];
    sensor = [50, 2*pi];
    agents(1,8) = Agent(8, x_agent, agentGrid, [width, height], sensor);
    agents(1,8).sense(occGrid);
end

if numAgents > 8
    x_agent = [144; 30; 0; 5];
    sensor = [50, 2*pi];
    agents(1,9) = Agent(9, x_agent, agentGrid, [width, height], sensor);
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
    for agent = agents
        % Start with the assumption we can't communicate with anyone
        agent.neighbors = Neighbor.empty;
        % Check comm, fuse maps, and get goals from other nearby agents
        for i=1 : length(agents)
            if agent.id ~= agents(i).id && checkLoS(agent.state(1:2), agents(i).state(1:2), occGrid)
                fusedGrid = fuseOccGrids(agent.occGrid, agents(i).occGrid);
                agent.occGrid = fusedGrid;
                agents(i).occGrid = fusedGrid;
                % Keep track of who we can talk to, and what their goal point is
                if ~isempty(agents(i).path)
                    npath = agents(i).path(end, :);
                else
                    npath = [];
                end

                %TODO Optimize the comm hopping next 30 lines
                store = true;
                for j=1 : length(agent.neighbors)
                    if agent.neighbors(j).id == agents(i).id
                        store = false;
                        break;
                    end
                end

                if store
                    agent.neighbors(end+1) = Neighbor(agents(i).id, npath);
                end

                for j=1 : length(agents(i).neighbors)
                    if agents(i).neighbors(j).id ~= agent.id
                        store = true;
                        for k=1 : length(agent.neighbors)
                            if agent.neighbors(k).id == agents(i).neighbors(j).id
                                store = false;
                                break;
                            end
                        end

                        if store
                            agent.neighbors(end+1) = agents(i).neighbors(j);
                        end
                    end
                end
            end
        end

        % Plan Path(s)
        if (mod(t, dt_plan) == 0)
            if ~isempty(agent.path)
                if ~frontierCheck(agent.path(end,2), agent.path(end,1), agent.occGrid)
                    agent.path = frontierPlan(agent.occGrid, agent.state(1:2), hblob, minObsDist, agent.neighbors, 5); % (occupancy grid, agent position, blob detector, minimum obstacle distance, figure number)
                end
            else
                agent.run = false;
            end
        end

        % Evolve state until the end of the path is no longer a frontier
        if size(agent.path,2) == 1
            agent.state(3) = wrapToPi(agent.state(3) + pi/2);
        else
            % Move the agent unless the path is empty, or if there's only
            % one point, which likely means the path is our current
            % position, which is saved to keep checking for frontiers
            if ~isempty(agent.path) && length(agent.path) > 2
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
    
    % Plot vehicle positions and belief grid
    if (mod(t, dt_plot) == 0)
        if plotFused
            plotFusedGrid(agents, 2);
        end
        if plotAgents
            plotAgentGrid(agents, 3);
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