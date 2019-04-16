classdef Artifact < handle
   
    properties
        pos
        type
        reportme = false
    end
    
    methods
        function obj = Artifact(pos, type)
            obj.pos = pos;
            obj.type = type;
        end
    end
end

