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
        
        view_norm = -[cos(azi), sin(azi), 1];
        
        % Frustum planes
        left_angle = pi - sensor_params.width/2;
        right_angle = sensor_params.width/2;
        top_angle = [];        
        bottom_angle = [];
        left_plane = [cos(left_angle), sin(left_angle), 1];
        right_plane = [cos(right_angle), sin(right_angle), 1];
        top_plane = [, , 1];
        bottom_plane = [, , 1];
        
        % Check for sightings
        for i=1:length(unseen_primitives)
            % Check radial range first
            
            
            % Check dot products with frustum planes
            
            
            % Check occlusion with occupancy grid
            
            
            
        end
    
end
end

