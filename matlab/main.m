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

% Choose which environment to explore
% options are demo, edgar and tunnel
environment = 'tunnel';
if strcmp(environment, 'demo')
    image = imread('example_environment.PNG');
    anchorState = [160; 20; 330*pi/180; 5];
    agentState = [144; 30; 0; 5]; % x (m), y (m), heading (rad from north), speed (m/s)
    artifact1pos = [90; 75];
    artifact2pos = [105; 250];
    artifact3pos = [220; 190];
elseif strcmp(environment, 'edgar')
    image = imread('Edgar_Mine_Editted.png');
    anchorState = [10; 730; 330*pi/180; 5];
    agentState = [10; 730; 0; 5];
    artifact1pos = [60; 65];
    artifact2pos = [240; 80];
    artifact3pos = [40; 160];
elseif strcmp(environment, 'tunnel')
    image = imread('tunnel_test.png');
    anchorState = [240; 130; 330*pi/180; 5];
    agentState = [280; 130; 0; 5];
    artifact1pos = [60; 65];
    artifact2pos = [240; 80];
    artifact3pos = [40; 160];
end

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

% Define ranges for limiting performance.  Use 300 for unlimited range.
sensorRange = 50;
commRange = 100;
commMapRange = 100;
commBuffer = 5;  % used for when to drop a beacon when near the end of comm range

% Define whether to drop beacons when approaching a corner, for both anchor and other beacon comm
% Normally set these the same, but available for testing
beaconsInAnchorCorner = 1;
beaconsInBeaconCorner = 1;

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
numAgents = 4;
agentSpread = 0;
% Number of beacons per robot
numBeacons = 4;

%% Generate robot(s)
x_agent = agentState;
sensor = [sensorRange, 2*pi];
agentGrid = 0.5*ones(m,n); % completely unknown
agents(1,1) = Agent(1, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
agents(1,1).sense(occGrid);

if agentSpread; x_agent = [172; 240; pi; 5]; end
sensor = [sensorRange, 2*pi];
agents(1,2) = Agent(2, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
agents(1,2).sense(occGrid);

if numAgents > 2
    if agentSpread; x_agent = [100; 200; pi; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,3) = Agent(3, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,3).sense(occGrid);
end

if numAgents > 3
    if agentSpread; x_agent = [120; 50; 270*pi/180; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,4) = Agent(4, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,4).sense(occGrid);
end

if numAgents > 4
    if agentSpread; x_agent = [140; 20; 0; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,5) = Agent(5, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,5).sense(occGrid);
end

if numAgents > 5
    if agentSpread; x_agent = [160; 230; pi; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,6) = Agent(6, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,6).sense(occGrid);
end

if numAgents > 6
    if agentSpread; x_agent = [110; 210; pi; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,7) = Agent(7, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,7).sense(occGrid);
end

if numAgents > 7
    if agentSpread; x_agent = [110; 70; 270*pi/180; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,8) = Agent(8, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,8).sense(occGrid);
end

if numAgents > 8
    if agentSpread; x_agent = [144; 30; 0; 5]; end
    sensor = [sensorRange, 2*pi];
    agents(1,9) = Agent(9, 'robot', x_agent, agentGrid, [width, height], sensor, numBeacons);
    agents(1,9).sense(occGrid);
end

% Add all of the beacons as additional agents, with no position currently
if numBeacons
    for i=length(agents)+1 : (numBeacons * length(agents)) + length(agents) + 1
        agents(1, i) = Agent(i, 'beacon', [0; 0; 0; 0], agentGrid, [width, height], [0, 0], 0);
        agents(1, i).path = [];
    end
end

anchor = Agent(1, 'anchor', anchorState, agentGrid, [width, height], sensor, 0);
for agent = agents
    anchor.neighbors(end+1) = Neighbor(anchor.id, agent.id, agent.type, agent.state(1:2), [], 0);
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
    artifacts(1,1) = Artifact(artifact1pos, 'flashlight');
    artifacts(1,2) = Artifact(artifact2pos, 'backpack');
    artifacts(1,3) = Artifact(artifact3pos, 'methane');
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
        % Skip beacons that haven't been deployed
        if strcmp(agent.type, 'beacon') && isequal(agent.state, [0;0;0;0])
            continue;
        end

        % Detect artifacts first so we can pass that information along immediately
        agent.detect(artifacts);

        % Start with the assumption we can't communicate with anyone
        agent.neighbors = Neighbor.empty;
        anchor.neighbors(agent.id).incomm = false;

        % Check comm, fuse maps, and get goals from other nearby agents
        for i=1 : length(agents)
            % Skip beacons that haven't been deployed
            if strcmp(agents(i).type, 'beacon') && isequal(agents(i).state, [0;0;0;0])
                continue;
            end

            % Make sure the agents are at least within comm range and line of sight
            dist = sqrt(sum((agent.state(1:2) - agents(i).state(1:2)) .^ 2));
            if agent.id ~= agents(i).id && dist < commRange && checkLoS(agent.state(1:2), agents(i).state(1:2), occGrid)
                % To fuse maps we need to be inside the commMapRange
                if dist < commMapRange
                    fusedGrid = fuseOccGrids(agent.occGrid, agents(i).occGrid);
                    agent.occGrid = fusedGrid;
                    agents(i).occGrid = fusedGrid;
                end

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
                % TODO This doesn't appear to be needed except in rare cases at the end.  Consider removing.
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
                    agent.neighbors(end+1) = Neighbor(agent.id, agents(i).id, agents(i).type, agents(i).state(1:2), npath, agents(i).cost);
                end

                % Get our neighbor's neighbors
                for j=1 : length(agents(i).neighbors)
                    % Make sure we're not looking at ourself
                    if agents(i).neighbors(j).id ~= agent.id
                        store = true;
                        % Check that we haven't already stored this neighbor
                        for k=1 : length(agent.neighbors)
                            if agent.neighbors(k).id == agents(i).neighbors(j).id
                                store = false;
                                break;
                            end
                        end

                        % For direct neighbors cid is agents(i).id, otherwise this is a neighbor of a neighbor
                        cur_agent = agents(i).neighbors(j).cid;
                        test_agent = agents(i).neighbors(j).id;

                        % Make sure that these two agents can actually still talk
                        % This is more computation than is efficient, but it's the cleanest way to do it
                        distn = sqrt(sum((agents(cur_agent).state(1:2) - agents(test_agent).state(1:2)) .^ 2));
                        if store && distn < commRange && checkLoS(agents(cur_agent).state(1:2), agents(test_agent).state(1:2), occGrid)
                            agent.neighbors(end+1) = agents(i).neighbors(j);
                            % Clear the replan flag for neighbors that haven't reset yet
                            agent.neighbors(end).replan = false;
                            % Need to check the current information for the neighbors because it may have changed since
                            % our neighbor received it, depending on the order of the neighbors
                            if ~isempty(agents(test_agent).path)
                                agent.neighbors(end).update(agents(test_agent).state(1:2), agents(test_agent).path(end, :), agents(test_agent).cost);
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
        dista = sqrt(sum((agent.state(1:2) - anchor.state(1:2)) .^ 2));
        checkAnchorComm = dista < commRange && checkLoS(agent.state(1:2), anchor.state(1:2), occGrid);
        if checkAnchorComm
            % To fuse maps we need to be inside the commMapRange
            if dista < commMapRange
                fusedGrid = fuseOccGrids(anchor.occGrid, agent.occGrid);
                anchor.occGrid = fusedGrid;
                agent.occGrid = fusedGrid;
            end

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
        elseif strcmp(agent.type, 'robot')
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

        % Decide whether to drop a beacon, only if we have one to drop
        if agent.beacons > 0
            dropBeacon = 0;
            beaconsInRange = 0;

            % The path may start behind, so we can't just look at the first couple of points
            pathlength = size(agent.path, 1);
            if pathlength > 30
                pathlength = 30;
            end

            % Look at each active beacon
            for beacon = agents
                if strcmp(beacon.type, 'beacon') && ~isequal(beacon.state, [0;0;0;0])
                    % Check to see if we can talk to a beacon, and therefore the anchor
                    distb = sqrt(sum((agent.state(1:2) - beacon.state(1:2)) .^ 2));
                    if distb < commRange && checkLoS(agent.state(1:2), beacon.state(1:2), occGrid)
                        beaconsInRange = beaconsInRange + 1;

                        % Drop a beacon if we're in the buffer range
                        if distb > (commRange - commBuffer)
                            dropBeacon = 1;
                        elseif beaconsInBeaconCorner
                            % Look ahead at the path to see if we'll go around a corner, and drop a beacon if so
                            for path = agent.path(1:pathlength,:).'
                                if ~checkLoS(path, beacon.state(1:2), occGrid)
                                    dropBeacon = 1;
                                    break;
                                end
                            end
                        end
                    end
                end
            end

            % If there are no beacons in range, but we're talking to the anchor, check if we should drop
            if ~beaconsInRange && checkAnchorComm
                % Drop if we're in the buffer zone
                if dista > (commRange - commBuffer)
                    dropBeacon = 1;
                elseif beaconsInAnchorCorner
                    % Drop if we're about to go around a corner, as above
                    for path = agent.path(1:pathlength,:).'
                        if ~checkLoS(path, anchor.state(1:2), occGrid)
                            dropBeacon = 1;
                            break;
                        end
                    end
                end
            elseif beaconsInRange > 1
                dropBeacon = 0;
            end

            if dropBeacon
                % Find the first available beacon
                for beacon = agents
                    if strcmp(beacon.type, 'beacon') && isequal(beacon.state, [0;0;0;0])
                        % Drop it
                        beacon.state = agent.state;
                        agent.beacons = agent.beacons - 1;
                        break;
                    end
                end
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
        if strcmp(agent.type, 'robot') && occGrid(floor(agent.state(2)), floor(agent.state(1)))
            disp(['Warning: Vehicle ', num2str(agent.id), ' collided with environment']);
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