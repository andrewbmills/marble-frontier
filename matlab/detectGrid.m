function [artifacts] = detectGrid(state, trueArtifacts, artifacts, agentGrid, gridDims, sensor)
[m, n] = size(agentGrid);
xGrid = state(1)/gridDims(1);
yGrid = state(2)/gridDims(2);
heading = state(3)*180/pi;
r = sensor(1) / 2; % sensing radius
dtheta = sensor(2)*180/pi; % sensor sweep angle

% Calculate which grid cells the sensor can detect
x_min = max([round(xGrid-r/gridDims(1)),1]);
x_max = min([round(xGrid+r/gridDims(1)),n]);
y_min = max([round(yGrid-r/gridDims(2)),1]);
y_max = min([round(yGrid+r/gridDims(2)),m]);

% Loop through the true artifacts to see if they can be detected, which is
% much faster than looking at everything the agent can see to see if an
% artifact is in the sensor view
for artifact = trueArtifacts
    run = true;
    % Check to see if the artifact is already stored, and skip if so
    for aa = artifacts
        if aa.pos == artifact.pos
            run = false;
            break;
        end
    end
    
    if run
        % Check if the artifact is in the sensor view, otherwise do nothing
        x = artifact.pos(1);
        y = artifact.pos(2);
        angle = atan2(x-xGrid, y-yGrid)*180/pi;
        if (x <= x_max) && (x >= x_min) && (y <= y_max) && (y >= y_min) && (abs(angle_diff(heading, angle)) <= dtheta)
            % Check for occlusion
            [x_ind, y_ind] = bresenham(xGrid, yGrid, x, y);
            occluded = 0;
            for ii = 1:(length(x_ind)-1)
                if agentGrid(y_ind(ii), x_ind(ii)) == 1
                    occluded = 1;
                end
            end
            if ~occluded
                artifacts(end+1) = artifact;
            end
        end
    end
end

end

function [ d ] = angle_diff( a, b )
% Computes a-b, preserving the correct sign. (in degrees)
a = mod(3600000 + a, 360);
b = mod(3600000 + b, 360);
d = a - b;
d = mod(d+180, 360) - 180;
end