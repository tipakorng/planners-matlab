classdef RrtNode < handle
    properties
        value
        parent
        edges
        children
    end
    
    methods
        function self = RrtNode(value, parent)
            self.value = value;
            self.parent = parent;
            self.edges = [];
            self.children = [];
        end
    
    end
end