function [check] = frontierCheck(i,j,occGrid)
[m,n] = size(occGrid);
i_min = max([i-1, 1]);
j_min = max([j-1, 1]);
i_max = min([i+1, m]);
j_max = min([i+1, n]);
localGrid = occGrid(i_min:i_max, j_min:j_max);
if sum(localGrid(:) == 9)
    check = 1;
else
    check = 0;
end
end

