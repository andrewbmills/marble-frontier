function sight = raycast(p1, p2, occGrid)
sight = 1;
if length(p1) == 2
    [ind_x, ind_y] = bresenham(p1(1), p2(1), p1(2), p2(2));
    for i=1:length(ind_x)
        if occGrid(ind_x(i), ind_y(i)) > 0.2
            sight = 0;
        end
    end
elseif length(p1) == 3
     [ind_x, ind_y, ind_z] = bresenham_line3d(p1, p2);
    for i=1:(length(ind_x)-1)
        if occGrid(ind_x(i), ind_y(i), ind_z(i)) > 0.2
            sight = 0;
        end
    end
end
end

