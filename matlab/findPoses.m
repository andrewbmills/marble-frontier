function [poses] = findPoses(primitives, occGrid, sensor_params)
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

[m, n, p] = size(occGrid);
unseen_primitives = primitives;

% Frustum planes in camera frame
left_plane = [sin(sensor_params.width/2); -cos(sensor_params.width/2); 0];
right_plane = [sin(sensor_params.width/2); cos(sensor_params.width/2); 0];
top_plane = [sin(sensor_params.elevation/2), 0, -cos(sensor_params.elevation/2)];
bottom_plane = [sin(sensor_params.elevation/2), 0, cos(sensor_params.elevation/2)];
front_plane = [-1; 0; 0];
back_plane = [1; 0; 0];
Frustum_normals = [front_plane, back_plane, left_plane, right_plane, top_plane, bottom_plane];

% Initialize poses set count
num_poses = 1;

while sum(unseen_primitives(:)) > 0
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
        
    % Check for sightings given the occGrid
        % Check to see if p_sample is in an occupied voxel
        if occGrid(round(p_sample(1)), round(p_sample(2)), round(p_sample(3)))
            continue
        end
        
        % Calculate the yaw angle so the pose faces sample primitive
        % centroid.
        yaw = wrapTo2pi(pi + azi); % CCW from x.
        
        % Rotation matrix given yaw angle
        R = [cos(yaw), -sin(yaw), 0; sim(yaw), cos(yaw), 0; 0, 0, 1];
        
        % Rotate frustum normals
        Frustum_local = R*Frustum_normals;
        
        % Check for occlusion to the generating primitive
        if ~raycast(p_sample, unseen_primitives(id).centroid, occGrid)
            continue
        else
            poses(num_poses).pose = [p_sample, yaw];
            poses(num_poses).sightings = id;
        end
        
    % Check for sightings
    for i=1:length(primitives)
        if i == id
            continue
        end
        
        % Calculate vector from camera frame origin to primitive
        p_primitive = primitive(i).centroid;
        v_sight = p_primitive - p_sample;

        % Check radial range first
        r = norm(v_sight);
        if (r > sensor_params.r_max || r < sensor_params.r_min)
            continue
        end

        % Check dot products with frustum planes
        v_sight_cam = R*v_sight;
        if sum((v_sight_cam'*Frustum_normals) < 0) > 0
            continue
        end

        % Check occlusion with occupancy grid
        if ~raycast(p_sample, p_primitive, occGrid)
            continue
        end

        % Add primitive to sightings list for this pose
        poses(num_poses).sightings(end+1) = primitive(i).id;
    end
    
    % 
    
end
end

