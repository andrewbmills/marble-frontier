%% Initialize an occupancy grid matrix in 3D
clear all
close all

% Define the xyz limits of the occGrid
x_range = 50;
y_range = 50;
z_range = 35;
[Y X Z] = meshgrid(1:y_range, 1:x_range, 1:z_range);

% Let's start with a rectangular hallway for simplicity
omap = 0.5*ones(x_range, y_range, z_range);
omap(1:40, 10:40, 1) = 1;
omap(1:40, 10:40, 20) = 1;
omap(1:40, 10, 1:20) = 1;
omap(1:40, 40, 1:20) = 1;
omap(1:40, 11:39, 2:19) = 0;
omap_data = omap;
omap_data(omap_data <= 0.6) = NaN;

scatter3(X(:), Y(:), Z(:), 50, omap_data(:), 'filled');
xlabel('x');
ylabel('y');
zlabel('z');

%% Find the frontier
frontier = zeros(x_range, y_range, z_range);
frontier(41, 11:39, 2:19) = 1;
frontier_data = frontier;
frontier_data(frontier_data == 0) = NaN;
hold on
% scatter3(X(:), Y(:), Z(:), 50, 2*frontier_data(:), 'filled');

%% Cluster the frontier
frontier_clustered = greedycluster(frontier, 4);
frontier_data = frontier_clustered + 1;
frontier_data(frontier_data == 1) = NaN;
scatter3(X(:), Y(:), Z(:), 100, frontier_data(:), 'filled');
colormap 'hsv'
hold off

%% Find poses to view each cluster
% Arrange the frontier_clustered matrix into an array of primitive
% structures
figure
scatter3(X(:), Y(:), Z(:), 50, omap_data(:), 'filled');
hold on
xlabel('x');
ylabel('y');
zlabel('z');
for num_clust=1:max(frontier_clustered(:))
    % Find all of the current cluster indices
    indices = find(frontier_clustered == num_clust);
    [x, y, z] = ind2sub(size(frontier_clustered), indices);
    centroid = mean([x,y,z], 1);
    primitives(num_clust).centroid = centroid;
    primitives(num_clust).id = num_clust;
    primitives(num_clust).members = [x, y, z];
    scatter3(centroid(1), centroid(2), centroid(3), 100, num_clust, 'filled');
end

sensor.r_min = 5;
sensor.r_max = 15;
sensor.width = pi/2; %(45 degrees);
sensor.elevation = pi/3; %(30 degrees);

poses = findPoses(primitives, omap, sensor, 1);

