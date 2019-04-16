function plotFusedGrid(agents, anchor, k)
    fusedArtifacts = agents(1).artifacts;
    if length(agents) == 1
        % For one agent, just use that agents occgrid
        fusedGrid = agents(1).occGrid;
    else
        % For multiple agents, first fuse the first two occgrids, then fuse
        % that one with subsequent agents' occgrids
        % Also fuse the artifacts so we don't render them multiple times
        fusedGrid = fuseOccGrids(agents(1).occGrid, agents(2).occGrid);
        fusedArtifacts = fuseArtifacts(agents(2).artifacts, fusedArtifacts);

        for i=3 : length(agents)
            fusedGrid = fuseOccGrids(fusedGrid, agents(i).occGrid);
            fusedArtifacts = fuseArtifacts(agents(i).artifacts, fusedArtifacts);
        end
    end
    
    figure(k)
    h = pcolor(fusedGrid);
    hold on
    
    % Plot the anchor node
    plot(anchor.state(1)+0.5, anchor.state(2)+0.5, 'g*');
    text(anchor.state(1)+1.5, anchor.state(2)+1.5, 'A', 'Color', 'green', 'FontSize', 12)

    for agent = agents
        plot(agent.state(1)+0.5, agent.state(2)+0.5, 'r*');
        text(agent.state(1)+1.5, agent.state(2)+1.5, num2str(agent.id), 'Color', 'red', 'FontSize', 12)
        if ~isempty(agent.path)
            plot(agent.path(:,1)+0.5, agent.path(:,2)+0.5, 'r');
        end
        plot(agent.stateHistory(:,2)+0.5, agent.stateHistory(:,3)+0.5, 'g-.');
    end
    
    for artifact = fusedArtifacts
        plot(artifact.pos(1), artifact.pos(2), 'w+');
        text(artifact.pos(1)+1.5, artifact.pos(2)+1.5, artifact.type, 'Color', 'white', 'FontSize', 12);
    end

    [m, n] = size(fusedGrid);
    axis([1 n 1 m]);
    set(h, 'EdgeColor', 'none');
    set(gcf, 'Position', [1, 1, 1080, 1080]);
    axis equal
    axis tight
    tightfig;
    hold off
        
end