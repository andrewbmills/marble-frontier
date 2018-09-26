function occGrid3 = fuseOccGrids(occGrid1, occGrid2)
% This function currently assumes occGrid1 and occGrid2 have no noise
% relative to one another.
occGrid3 = 0.5*ones(size(occGrid1));
occGrid3(occGrid1 == 1) = 1;
occGrid3(occGrid2 == 1) = 1;
occGrid3(occGrid1 == 0) = 0;
occGrid3(occGrid2 == 0) = 0;
end