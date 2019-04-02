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
[m, n] = size(occGrid);

%% Some global variables to define behavior
global deconfliction;
deconfliction = 2; % Deconfliction level: 0-None, 1-Finders Keepers, 2-Lowest Cost
global gridPlots;
gridPlots = 0; % Plot the frontier plots or not
% These used to be globals, don't need to be
plotFused = 1; % Plot all robots on one graph
plotAgents = 0; % Plot individual robot views on single graph, automatically sized
plotAnchor = 1; % Plot what the anchor node can see
plotRealtime = 0; % Plot the agent paths as they are calculated instead of all at once
plotOccGrid = 0; % Plot the actual occgrid for reference
enableArtifacts = 1; % Whether to use artifacts and artifact detection

if plotOccGrid
    h1 = pcolor(occGrid);
    set(h1, 'EdgeColor', 'none');
end

% Placement of anchor plot depends on whether we're showing the full fused map as well
if plotFused
    anchorPlotNum = 3;
else
    anchorPlotNum = 2;
end

%% Use this to decide how many agents to run, and whether they all start from the same location
numAgents = 9;
agentSpread = 0;

%% Generate robot(s)
x_agent = [144; 30; 0; 5]; % x (m), y (m), heading (rad from north), speed (m/s)
sensor = [50, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agents(1,1) = Agent(1, x_agent, agentGrid, [width, height], sensor);
agents(1,1).sense(occGrid);

if agentSpread; x_agent = [172; 240; pi; 5]; end
sensor = [50, 2*pi];
agents(1,2) = Agent(2, x_agent, agentGrid, [width, height], sensor);
agents(1,2).sense(occGrid);

if numAgents > 2
    if agentSpread; x_agent = [100; 200; pi; 5]; end
    sensor = [50, 2*pi];
    agents(1,3) = Agent(3, x_agent, agentGrid, [width, height], sensor);
    agents(1,3).sense(occGrid);
end

if numAgents > 3
    if agentSpread; x_agent = [120; 50; 270*pi/180; 5]; end
    sensor = [50, 2*pi];
    agents(1,4) = Agent(4, x_agent, agentGrid, [width, height], sensor);
    agents(1,4).sense(occGrid);
end

if numAgents > 4
    if agentSpread; x_agent = [140; 20; 0; 5]; end
    sensor = [50, 2*pi];
    agents(1,5) = Agent(5, x_agent, agentGrid, [width, height], sensor);
    agents(1,5).sense(occGrid);
end

if numAgents > 5
    if agentSpread; x_agent = [160; 230; pi; 5]; end
    sensor = [50, 2*pi];
    agents(1,6) = Agent(6, x_agent, agentGrid, [width, height], sensor);
    agents(1,6).sense(occGrid);
end

if numAgents > 6
    if agentSpread; x_agent = [110; 210; pi; 5]; end
    sensor = [50, 2*pi];
    agents(1,7) = Agent(7, x_agent, agentGrid, [width, height], sensor);
    agents(1,7).sense(occGrid);
end

if numAgents > 7
    if agentSpread; x_agent = [110; 70; 270*pi/180; 5]; end
    sensor = [50, 2*pi];
    agents(1,8) = Agent(8, x_agent, agentGrid, [width, height], sensor);
    agents(1,8).sense(occGrid);
end

if numAgents > 8
    if agentSpread; x_agent = [144; 30; 0; 5]; end
    sensor = [50, 2*pi];
    agents(1,9) = Agent(9, x_agent, agentGrid, [width, height], sensor);
    agents(1,9).sense(occGrid);
end

anchor = Agent(1, [160; 20; 330*pi/180; 5], agentGrid, [width, height], sensor);
for agent = agents
    anchor.neighbors(end+1) = Neighbor(agent.id, agent.state(1:2), [], 0);
end

if plotFused
    plotFusedGrid(agents, anchor, 2);
end
if plotAgents
    plotAgentGrid(agents, 3);
end
if plotAnchor
    plotAnchorGrid(anchor, anchorPlotNum);
end

%% Generate artifacts
if enableArtifacts
    artifacts(1,1) = Artifact([90; 75], 'flashlight');
    artifacts(1,2) = Artifact([105; 250], 'backpack');
    artifacts(1,3) = Artifact([220; 190], 'methane');
else
    artifacts = [];
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
        % Detect artifacts first so we can pass that information along immediately
        agent.detect(artifacts);

        % Start with the assumption we can't communicate with anyone
        agent.neighbors = Neighbor.empty;
        anchor.neighbors(agent.id).incomm = false;

        % Check comm, fuse maps, and get goals from other nearby agents
        for i=1 : length(agents)
            if agent.id ~= agents(i).id && checkLoS(agent.state(1:2), agents(i).state(1:2), occGrid)
                fusedGrid = fuseOccGrids(agent.occGrid, agents(i).occGrid);
                agent.occGrid = fusedGrid;
                agents(i).occGrid = fusedGrid;

                fusedArtifacts = fuseArtifacts(agent.artifacts, agents(i).artifacts);
                agent.artifacts = fusedArtifacts;
                agents(i).artifacts = fusedArtifacts;

                % Get the goal point of the agent
                if ~isempty(agents(i).path)
                    npath = agents(i).path(end, :);
                else
                    npath = [];
                end

                %TODO Optimize the comm hopping next 30 lines
                store = true;
                for j=1 : length(agent.neighbors)
                    % If it's already in our list, we just need to make sure that it's up-to-date
                    if agent.neighbors(j).id == agents(i).id
                        % Clear the replan flag for neighbors that haven't reset yet
                        agent.neighbors(j).replan = false;
                        agent.neighbors(j).cost = agents(i).cost;
                        % Need to check the current goal point for the neighbors because it may have changed since
                        % our neighbor received it, depending on the order of the neighbors
                        if ~isempty(agents(i).path)
                            agent.neighbors(j).goal = agents(i).path(end, :);
                        else
                            agent.neighbors(j).goal = [];
                        end
                        store = false;
                        break;
                    end
                end

                % Save our direct neighbor, unless deconfliction level 0
                if deconfliction > 0 && store
                    agent.neighbors(end+1) = Neighbor(agents(i).id, agents(i).state(1:2), npath, agents(i).cost);
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
                            % Clear the replan flag for neighbors that haven't reset yet
                            agent.neighbors(end).replan = false;
                            % Need to check the current information for the neighbors because it may have changed since
                            % our neighbor received it, depending on the order of the neighbors
                            if ~isempty(agents(agents(i).neighbors(j).id).path)
                                agent.neighbors(end).update(agents(agents(i).neighbors(j).id).state(1:2), agents(agents(i).neighbors(j).id).path(end, :), agents(agents(i).neighbors(j).id).cost);
                            else
                                agent.neighbors(end).goal = [];
                            end
                        end
                    end
                end
            end
        end

        anchorGoal = [];
        % Check comm with anchor node and fuse maps
        if checkLoS(agent.state(1:2), anchor.state(1:2), occGrid)
            fusedGrid = fuseOccGrids(anchor.occGrid, agent.occGrid);
            anchor.occGrid = fusedGrid;
            agent.occGrid = fusedGrid;

            % Flag artifacts as reported now that we've talked with the anchor
            for artifact = agent.artifacts
                artifact.reportme = false;
            end
            fusedArtifacts = fuseArtifacts(anchor.artifacts, agent.artifacts);
            anchor.artifacts = fusedArtifacts;
            agent.artifacts = fusedArtifacts;

            if ~isempty(agent.path)
                npath = agent.path(end, :);
            else
                npath = [];
            end
            anchor.neighbors(agent.id).update(agent.state(1:2), npath, agent.cost);
            anchor.neighbors(agent.id).history(agent.stateHistory, agent.path);
            for neighbor = agent.neighbors
                anchor.neighbors(neighbor.id).update(neighbor.pos, neighbor.goal, neighbor.cost);
                anchor.neighbors(neighbor.id).history(agents(neighbor.id).stateHistory, agents(neighbor.id).path);
            end
        else
            % Check if any artifacts need to be reported, and if so, plan to go there
            for artifact = agent.artifacts
                if artifact.reportme
                    anchorGoal = sub2ind(size(occGrid), anchor.state(2), anchor.state(1));
                    break;
                end
            end
        end

        % Plan Path(s)
        if (mod(t, dt_plan) == 0)
            if ~isempty(agent.path)
                if ~frontierCheck(agent.path(end,2), agent.path(end,1), agent.occGrid)
                    [agent.path, agent.cost] = frontierPlan(agent.occGrid, agent.state(1:2), anchorGoal, hblob, minObsDist, agent.neighbors, 5); % (occupancy grid, agent position, blob detector, minimum obstacle distance, figure number)
                    % Check to see if any neighbors need to be replanned because we took their goal point
                    for neighbor = agent.neighbors
                        % If the id is after us, they'll be forced to
                        % replan due to cost anyway
                        % if neighbor.replan && neighbor.id < agent.id
                        if neighbor.replan
                            id = neighbor.id;
                            agents(id).path = [round(agents(id).state(1)), round(agents(id).state(2))];
                            agents(id).cost = 0;
                        end
                    end
                end
            else
                agent.run = false;
            end
        end

        % Evolve state until the end of the path is no longer a frontier
        if size(agent.path, 1) == 1 && agent.path(1) ~= round(agent.state(1)) && agent.path(2) ~= round(agent.state(2))
            agent.state(3) = wrapToPi(agent.state(3) + pi/2);
        elseif size(agent.path, 1) > 1
            % Move the agent unless the path is empty, or if there's only
            % one point, which likely means the path is our current
            % position, which is saved to keep checking for frontiers
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
        
        if plotRealtime && plotFused && (mod(t, dt_plot) == 0)
            plotFusedGrid(agents, anchor, 2);
        end
    end

    % Plot vehicle positions and belief grid
    if (mod(t, dt_plot) == 0)
        if ~plotRealtime && plotFused
            plotFusedGrid(agents, anchor, 2);
        end
        if plotAgents
            plotAgentGrid(agents, 3);
        end
        if plotAnchor
            plotAnchorGrid(anchor, anchorPlotNum);
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
    if sum([agents.run]) == 1
        disp(t)
    end
end

disp(t)

if plotFused
    plotFusedGrid(agents, anchor, 2);
end
if plotAgents
    plotAgentGrid(agents, 3);
end
if plotAnchor
    plotAnchorGrid(anchor, anchorPlotNum);
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