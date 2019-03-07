classdef Neighbor
    %NEIGHBOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        goal
    end
    
    methods
        function obj = Neighbor(id, goal)
            obj.id = id;
            obj.goal = goal;
        end
        
    end
end

