function [path] = findPath(reachGrid, goal, start)
i = goal(1);
j = goal(2);
i_start = start(1);
j_start = start(2);
figure(10)
[px,py] = gradient(reachGrid);
modGrid = sqrt(px.^2 + py.^2);
% contour(reachGrid);
% hold on
% quiver(-px./modGrid,-py./modGrid);
% plot(j, i, 'r*');
% plot(j_start, i_start, 'g*');
path = [j, i];
while true
%     plot(path(:,1), path(:,2), 'r');
    i_next = round(i - py(i,j)/modGrid(i,j));
    j_next = round(j - px(i,j)/modGrid(i,j));
    i = i_next;
    j = j_next;
    path = [j, i; path];
    if (i == i_start) && (j == j_start)
        break
    end
    if size(path,1) >= 10000
        error('no safe path to frontier')
        break
    end
end
% plot(path(:,1), path(:,2), 'r');
% hold off
end

