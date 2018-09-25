function [check] = frontierCheck(i,j,occGrid)
[m,n] = size(occGrid);
i_min = max([i-1, 1]);
j_min = max([j-1, 1]);
i_max = min([i+1, m]);
j_max = min([j+1, n]);
localGrid = occGrid(i_min:i_max, j_min:j_max);
if occGrid(i_min, j) == 0.5 || occGrid(i_max, j) == 0.5 || occGrid(i, j_min) == 0.5 || occGrid(i, j_max) == 0.5
    check = 1;
else
    check = 0;
end
end

