%% Initialize an occupancy grid matrix in 3D
% Define the xyz limits of the occGrid
x_range = 50;
y_range = 50;
z_range = 35;
[X Y Z] = meshgrid(1:y_range, 1:x_range, 1:z_range);

% Let's start with a rectangular hallway for simplicity
omap = zeros(x_range, y_range, z_range);
omap(1:40, 10:40, 1) = 1;
omap(1:40, 10:40, 20) = 1;
omap(1:40, 10, 1:20) = 1;
omap(1:40, 40, 1:20) = 1;
omap_data = omap;
omap_data(omap_data == 0) = NaN;

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