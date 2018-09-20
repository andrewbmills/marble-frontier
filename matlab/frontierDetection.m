function [frontGrid] = frontierDetection(occGrid)
kernel = [1, 1, 1; 1, -81, 1; 1, 1, 1];
frontGrid = conv2(occGrid, kernel, 'same')>=9;
end

