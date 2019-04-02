function plotAnchorGrid(anchor, k)
    fusedGrid = anchor.occGrid;
    
    figure(k)
    h = pcolor(fusedGrid);
    hold on
    
    % Plot the anchor node
    plot(anchor.state(1)+0.5, anchor.state(2)+0.5, strcat('g*'));
    text(anchor.state(1)+1.5, anchor.state(2)+1.5, 'A', 'Color', 'green', 'FontSize', 12)

    for artifact = anchor.artifacts
        plot(artifact.pos(1), artifact.pos(2), 'w+');
        text(artifact.pos(1)+1.5, artifact.pos(2)+1.5, artifact.type, 'Color', 'white', 'FontSize', 12);
    end

    % Plot all of the agents that we have information about
    for agent = anchor.neighbors
        % Plot normal red if we have current information, yellow otherwise
        if agent.incomm
            color = 'red';
            scolor = 'r';
        else
            color = 'yellow';
            scolor = 'y';
        end
        plot(agent.pos(1)+0.5, agent.pos(2)+0.5, strcat(scolor, '*'));
        text(agent.pos(1)+1.5, agent.pos(2)+1.5, num2str(agent.id), 'Color', color, 'FontSize', 12)
        if ~isempty(agent.path)
            plot(agent.path(:,1)+0.5, agent.path(:,2)+0.5, scolor);
        end
        plot(agent.stateHistory(:,2)+0.5, agent.stateHistory(:,3)+0.5, 'g-.');
    end
    
    [m, n] = size(anchor.occGrid);
    axis([1 n 1 m]);
    set(h, 'EdgeColor', 'none');
    if k == 2
        set(gcf, 'Position', [1, 1, 1080, 1080]);
    else
        set(gcf, 'Position', [1080, 1, 1080, 1080]);
    axis equal
    axis tight
    tightfig;
    hold off
        
end