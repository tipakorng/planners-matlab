classdef RrtNode < handle
    properties
        value
        parent
        edges
        children
        cost
    end
    
    methods
        function self = RrtNode(value, parent)
            self.value = value;
            self.parent = parent;
            self.edges = [];
            self.children = [];
            self.cost = 0;
        end
    
    end
end