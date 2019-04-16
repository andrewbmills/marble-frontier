classdef Neighbor < handle
    %NEIGHBOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        pos
        goal
        cost
        stateHistory
        path = []
        replan = false
        incomm = false
    end
    
    methods
        function obj = Neighbor(id, pos, goal, cost)
            obj.id = id;
            obj.pos = pos;
            obj.goal = goal;
            obj.cost = cost;
            obj.stateHistory = [0, pos'];
        end
        
        function update(obj, pos, goal, cost)
            obj.pos = pos;
            obj.goal = goal;
            obj.cost = cost;
            obj.incomm = true;
        end

        function history(obj, stateHistory, path)
            obj.stateHistory = stateHistory;
            obj.path = path;
        end
    end
end

