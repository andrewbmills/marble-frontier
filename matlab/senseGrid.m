function [agentGrid] = senseGrid(state, occGrid, agentGrid, gridDims, sensor)
[m, n] = size(occGrid);
xGrid = state(1)/gridDims(1); % put 
yGrid = state(2)/gridDims(2);
heading = state(3)*180/pi;
r = sensor(1); % sensing radius
dtheta = sensor(2)*180/pi; % sensor sweep angle

% Calculate which grid cells the sensor can detect
j_min = max([round(xGrid-r/gridDims(1)),1]);
j_max = min([round(xGrid+r/gridDims(1)),n]);
i_min = max([round(yGrid-r/gridDims(2)),1]);
i_max = min([round(yGrid+r/gridDims(2)),m]);
localGrid = occGrid(i_min:i_max, j_min:j_max);
senseGrid = zeros(size(localGrid));
for i = i_min:i_max
    for j = j_min:j_max
        angle = atan2(j-xGrid, i-yGrid)*180/pi;
        if abs(angle_diff(heading, angle)) <= dtheta
            % Check for occlusion
            [x_ind, y_ind] = bresenham(xGrid, yGrid, j, i);
            occluded = 0;
            for ii = 1:(length(x_ind)-1)
                if occGrid(y_ind(ii), x_ind(ii)) == 1
                    occluded = 1;
                end
            end
            if ~occluded
                senseGrid(i-i_min+1, j-j_min+1) = 1;
            end
        end
    end
end

agentGrid(i_min:i_max, j_min:j_max) = (1-senseGrid).*...
    agentGrid(i_min:i_max, j_min:j_max) + senseGrid.*localGrid;

end

function [ d ] = angle_diff( a, b )
% Computes a-b, preserving the correct sign. (in degrees)
a = mod(3600000 + a, 360);
b = mod(3600000 + b, 360);
d = a - b;
d = mod(d+180, 360) - 180;
end