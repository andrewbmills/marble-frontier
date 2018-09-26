function [LoS] = checkLoS(p1, p2, occGrid)
[x_ind, y_ind] = bresenham(p1(1), p1(2), p2(1), p2(2));
LoS = true;
for i = 2:(length(x_ind)-1)
    if occGrid(y_ind(i), x_ind(i)) == 1
        LoS = false;
    end
end
end

