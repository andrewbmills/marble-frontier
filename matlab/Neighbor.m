classdef Neighbor < handle
    %NEIGHBOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        goal
        cost
        replan = false
    end
    
    methods
        function obj = Neighbor(id, goal, cost)
            obj.id = id;
            obj.goal = goal;
            obj.cost = cost;
        end
        
    end
end

