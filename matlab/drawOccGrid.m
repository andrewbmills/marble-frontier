function [occGrid] = drawOccGrid(gridSize, pointerSize)
%Prompts the user to draw an occupancy grid for robot planning.  The user
%may select pointerSize, which is the number of grid spaces each click
%fills, and pointerStyle which determines whether the pointer fills in
%circles or squares.  (Defaults are pointerSize = 1 and pointerStyle = 
%"Square").

m = gridSize(1);
n = gridSize(2);
occGrid = zeros(m,n);
figure(10);
pcolor(occGrid);
axis([1 n+1 1 m+1]);
% view(2);
% Make all the border cells occupied
occGrid(1,:) = 1;
occGrid(:,1) = 1;
occGrid(end,:) = 1;
occGrid(:,end) = 1;

while 1
    pt = ginput(1);
    if isempty(pt) || ~ishghandle(10)
        break
    else
        i = floor(pt(2));
        j = floor(pt(1));
        i_low = max([i-pointerSize+1,1]);
        j_low = max([j-pointerSize+1,1]);
        i_high = min([i+pointerSize-1,m]);
        j_high = min([j+pointerSize-1,n]);
        if pointerSize == 1
            occGrid(i_low:i_high, j_low:j_high) = ~occGrid(i_low:i_high, j_low:j_high);
        else
            occGrid(i_low:i_high, j_low:j_high) = 1;
        end
        pcolor(occGrid);
        axis([1 n+1 1 m+1]);
        %         view(2);
    end
end

save('lastOccGridDrawing', 'occGrid')

end
