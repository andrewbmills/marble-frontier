function plotAgentGrid(agents, k)
    figure(k)
    set(gcf, 'Position', [1080, 1, 1400, 1400]);
    al = length(agents);
    
    % Figure out the size of the plotting grid based on number of agents
    if mod(al,3) == 0 || al == 5
        rows = 3;
        cols = 3;
    elseif al == 8 || al == 7
        rows = 4;
        cols = 2;
    else
        rows = 2;
        cols = 2;
    end
    
    for i = 1 : al
        % Only plot the first 9 agents, due to subplot formatting
        % If we wanted more, we could make new figures for every 9
        if i <= 9
            subplot(rows, cols, i)
            agents(i).plot();
        end
    end
    tightfig;
end