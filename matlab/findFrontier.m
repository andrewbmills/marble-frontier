function [frontGrid] = findFrontier(occGrid)
[m, n] = size(occGrid);
frontGrid = zeros(m, n);
for i = 1:m*n
    if occGrid(i) == 0
        [j,k] = ind2sub([m,n], i);
        k_low = max([k-1,1]);
        j_low = max([j-1,1]);
        k_high = min([k+1,n]);
        j_high = min([j+1,m]);
        if occGrid(j_low, k) == 0.5
            frontGrid(j,k) = 1.0;
%         elseif occGrid(j_low, k_low) == 0.5
%             frontGrid(j,k) = 1.0;
%         elseif occGrid(j_low, k_high) == 0.5
%             frontGrid(j,k) = 1.0;
        elseif occGrid(j, k_low) == 0.5
            frontGrid(j,k) = 1.0;
        elseif occGrid(j, k_high) == 0.5
            frontGrid(j,k) = 1.0;
%         elseif occGrid(j_high, k_low) == 0.5
%             frontGrid(j,k) = 1.0;
        elseif occGrid(j_high, k) == 0.5
            frontGrid(j,k) = 1.0;
%         elseif occGrid(j_high, k_high) == 0.5
%             frontGrid(j,k) = 1.0;
        end
    end
end
end

