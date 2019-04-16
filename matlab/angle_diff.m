function [d] = angle_diff(a, b)
    % Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    % All angles are in degrees
    a = mod(360000 + a, 360);
    b = mod(360000 + b, 360);
    d = a - b;
    d = mod(d + 180, 360) - 180;
end

