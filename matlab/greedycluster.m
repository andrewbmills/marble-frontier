function [M2] = greedycluster(M1, r)
[m, n, p] = size(M1);
M2 = zeros(m,n,p);
r_ceil = ceil(r);
N_cluster = 1;

% For plotting
% [X_all,Y_all,Z_all] = meshgrid(1:n, 1:m, 1:p);
% hold off

% Compute the local cluster radius matrix
local_max = 2*r+1;
local_range = 1:local_max;
[X, Y, Z] = meshgrid(local_range, local_range, local_range);
c = r+1; % center of the local cluster
cluster_local = ((X - c).^2 + (Y - c).^2 + (Z - c).^2) <= r^2;

while (sum(M1(:)) > 0)
    % Generate a set of the unclustered points in M1
    unclustered = find(M1==1);
    
    % Sample an unclustered point from M1
    idx = unclustered(randi(length(unclustered)));
    [x, y, z] = ind2sub([m,n,p], idx);
    
    % Subsample the cluster matrix given the input matrix limits
    x_min = max(x - r_ceil, 1);
    x_max = min(x + r_ceil, m);
    y_min = max(y - r_ceil, 1);
    y_max = min(y + r_ceil, n);
    z_min = max(z - r_ceil, 1);
    z_max = min(z + r_ceil, p);
    x_range = x_min:x_max;
    y_range = y_min:y_max;
    z_range = z_min:z_max;
    
    % Trim the local cluster
    x_trim_min = max(r-x+2,1);
    y_trim_min = max(r-y+2,1);
    z_trim_min = max(r-z+2,1);
    x_trim_max = min(local_max + (m - (x + r)),local_max);
    y_trim_max = min(local_max + (n - (y + r)),local_max);
    z_trim_max = min(local_max + (p - (z + r)),local_max);
    x_trim = x_trim_min:x_trim_max;
    y_trim = y_trim_min:y_trim_max;
    z_trim = z_trim_min:z_trim_max;
    cluster_local_trimmed = cluster_local(x_trim, y_trim, z_trim);
    
    % Add cluster to the global one
    cluster = (zeros(m,n,p) == 1);
    cluster(x_range, y_range, z_range) = cluster_local_trimmed;
    M2(cluster) = N_cluster*M1(cluster) + M2(cluster);
    M1(cluster) = 0;
    
    % Plot the most recent cluster
%     M2_data = M2;
%     M2_data(M2_data == 0) = NaN;
%     scatter3(X_all(:), Y_all(:), Z_all(:), 50, M2_data(:), 'filled');
    
    N_cluster = N_cluster + 1;
end
end

