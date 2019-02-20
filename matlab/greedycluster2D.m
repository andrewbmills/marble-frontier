function [M2] = greedycluster2D(M1, r)
[m, n] = size(M1);
M2 = zeros(m,n);
r_ceil = ceil(r);
N_cluster = 1;
while (sum(M1(:)) > 0)
    % Generate a set of the unclustered points in M1
    unclustered = find(M1==1);
    
    % Sample an unclustered point from M1
    idx = unclustered(randi(length(unclustered)));
    [x, y] = ind2sub([m,n], idx);
    
    % Define an index radius around the sample point
    x_min = max(x - r_ceil, 1);
    x_max = min(x + r_ceil, m);
    y_min = max(y - r_ceil, 1);
    y_max = min(y + r_ceil, n);
    x_range = x_min:x_max;
    y_range = y_min:y_max;
    
    % Update M2 with the cluster number times all unclustered points in M1
    for i = x_range
        for j = y_range
            if norm([i,j] - [x, y]) < r
                M2(i,j) = M1(i,j)*N_cluster + M2(i,j);
                M1(i,j) = 0;
            end
        end
    end
    N_cluster = N_cluster + 1;
end
end

