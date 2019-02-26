function [poses] = findPoses(primitives, occGrid, sensor_params, redundancy)
% Input variables -
% primitives: (N length structure array)
%   primitives.id
%   primitives.members -
%       - [x,y,z] x M location and cluster number of every primitive member
%   primitives.centroid - 
%       - [x y z] location of the cluster centroid
%
% occGrid: (m x n x p)
%   Occupancy grid values in 3D
%
% sensor_params: (structure)
%   sensor_params.r_min
%   sensor_params.r_min
%   sensor_params.width (angle or size at r_min)
%   sensor_params.elevation (angle or size at r_min)
%
% Output variables -
% poses: (structure array)
%   Candidate poses for viewing primitives
%   poses.pose - 
%       - [x, y, z, yaw] (pitch should be 0)
%   poses.sightings - (1D vector)
%       - integer IDs of primitives viewable from this pose
% 

% Initialization
[m,n,p] = size(occGrid);
unseen_primitives = primitives;

% Frustum planes in camera frame
left_plane = [sin(sensor_params.width/2); -cos(sensor_params.width/2); 0];
right_plane = [sin(sensor_params.width/2); cos(sensor_params.width/2); 0];
top_plane = [sin(sensor_params.elevation/2); 0; -cos(sensor_params.elevation/2)];
bottom_plane = [sin(sensor_params.elevation/2); 0; cos(sensor_params.elevation/2)];
front_plane = [1; 0; 0];
Frustum_normals = [front_plane, left_plane, right_plane, top_plane, bottom_plane];

% Empty pose
first_pose = 1;
empty_pose.pose = [0, 0, 0, 0];
empty_pose.sightings = [0];

% Redundancy matrix
num_sightings = zeros(1,length(primitives));

% Plotting tools
[Y,X,Z] = meshgrid(1:n, 1:m, 1:p);
omap_data = zeros(size(occGrid));
omap_data(occGrid <= 0.6) = NaN;
figure
scatter3(X(:), Y(:), Z(:), 50, omap_data(:), 'filled');
hold on
h = plot3(0,0,0);
h2 = plot3(0,0,0);

% View pyramid points
p2 = sensor_params.r_max*[1; sin(sensor_params.width/2); sin(sensor_params.elevation/2)];
p3 = sensor_params.r_max*[1; -sin(sensor_params.width/2); sin(sensor_params.elevation/2)];
p4 = sensor_params.r_max*[1; -sin(sensor_params.width/2); -sin(sensor_params.elevation/2)];
p5 = sensor_params.r_max*[1; sin(sensor_params.width/2); -sin(sensor_params.elevation/2)];
Frustum_points = [p2, p3, p4, p5];

while ~isempty(unseen_primitives)
    % Choose a random unseen_primitive
    id = randi(length(unseen_primitives));
    
    % Sample a pose in spherical coords (radius, azimuth, and elevation)
    % that sees the primitive without obstacles
        % Uniformly sample a range value in [r_min, r_max]
        r = unifrnd(sensor_params.r_min, sensor_params.r_max);
        
        % Sample feasible elevations [elev_min, elev_max]
        elev = unifrnd(pi/2 - sensor_params.elevation/2, pi/2 + sensor_params.elevation/2);
        
        % Sample azimuth [0, 2*pi]
        azi = unifrnd(0, 2*pi);
        
        % Convert to x,y,z and check for sightings
        p_sample = unseen_primitives(id).centroid + r*...
            [sin(elev)*cos(azi), sin(elev)*sin(azi), cos(elev)];
        
    % Check for feasability of the current sighting
        % Check to see if p_sample is in the occGrid volume
        if sum([m,n,p] < p_sample) || sum(p_sample < 1)
            continue
        end
        % Check to see if p_sample is in an occupied voxel
        if occGrid(round(p_sample(1)), round(p_sample(2)), round(p_sample(3))) > 0.2
            continue
        end
        
        % Calculate the yaw angle so the pose faces sample primitive
        % centroid.
        yaw = wrapTo2Pi(pi + azi); % CCW from x.
        
        % Check for occlusion to the generating primitive
        if ~raycast(p_sample, unseen_primitives(id).centroid, occGrid)
            continue
        elseif (first_pose == 1)
            poses(1).pose = [p_sample, yaw];
            poses(1).sightings = id;
            first_pose = 0;
        else
            poses = [poses, empty_pose];
            poses(end).pose = [p_sample, yaw];
            poses(end).sightings = id;
        end
    
    % Rotation matrix given yaw angle
    R = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];

    % Rotate frustum normals
    Frustum_local = R*Frustum_normals;
    
    % Plot the view pyramid
    delete(h);
    Frustum_local_points = R*Frustum_points + repmat(p_sample', 1, 4);
    h = plot3([p_sample(1)*ones(1,4), Frustum_local_points(1,:); ...
        Frustum_local_points(1,:), Frustum_local_points(1,2:4), Frustum_local_points(1,1)], ...
        [p_sample(2)*ones(1,4), Frustum_local_points(2,:); ...
        Frustum_local_points(2,:), Frustum_local_points(2,2:4), Frustum_local_points(2,1)], ...
        [p_sample(3)*ones(1,4), Frustum_local_points(3,:) ; ...
        Frustum_local_points(3,:), Frustum_local_points(3,2:4), Frustum_local_points(3,1)], 'k');
    
    
    % Check for sightings
    for i=1:length(primitives)
        if i == id
            continue
        end
        
        % Calculate vector from camera frame origin to primitive
        p_primitive = primitives(i).centroid;
        v_sight = p_primitive - p_sample;
        
        % Plot the LoS from the camera to the primitive centroid
        delete(h2);
        h2 = plot3([p_sample(1); p_primitive(1)], [p_sample(2); ...
            p_primitive(2)], [p_sample(3); p_primitive(3)], 'r');

        % Check radial range first
        r = norm(v_sight);
        if (r > sensor_params.r_max || r < sensor_params.r_min)
            continue
        end

        % Check dot products with frustum planes
        if sum((v_sight*Frustum_local) < 0) > 0
            continue
        end

        % Check occlusion with occupancy grid
        if ~raycast(p_sample, p_primitive, occGrid)
            continue
        end

        % Add primitive to sightings list for this pose
        poses(end).sightings(end+1) = primitives(i).id;
        num_sightings(primitives(i).id) = num_sightings(primitives(i).id) + 1;
    end
    
    % Remove sighted primitives from unseen_primitives after hitting the
    % redundancy requirement.
    unseen_primitives = primitives(num_sightings < redundancy);
    
end
end

