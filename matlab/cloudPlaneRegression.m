%% Estimate the normal of a plane and its residual from a list of 3d points
points = [139.83, 39.81, -3.56; ...
          138.51, 42.68, -3.28; ...
          139.56, 42.84, -2.78; ...
          140.62, 41.65, -4.27]; % meters
robot = [139.87, 44.96, -3.29]; % meters
cost = [25.08; 13.76; 11.28; 18.06];
voxel_size = 0.2;

euclidean_dist = sqrt(sum((repmat(robot, 4, 1) - points).^2, 2))/voxel_size
cost