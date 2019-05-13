classdef Neighbor < handle
    %NEIGHBOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cid  % id of the node that's actually in direct comm
        id
        type
        pos
        goal
        cost
        stateHistory
        path = []
        replan = false
        incomm = true  % if we just created a neighbor, then we must be in comm!
    end
    
    methods
        function obj = Neighbor(cid, id, type, pos, goal, cost)
            obj.cid = cid;
            obj.id = id;
            obj.type = type;
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

