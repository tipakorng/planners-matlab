classdef RrtNode < handle
    properties
        
        % vertex is a point in an n-dimensional Euclidean space
        vertex
        
        % parent is the parent of this node
        parent
        
        % edges connect this node and its children
        edges
        
        % children are the children node
        children
        
        % cost is the cost of reaching this node from the root
        cost
    end
    
    methods
        function self = RrtNode(vertex, parent)
            self.vertex = vertex;
            self.parent = parent;
            self.edges = [];
            self.children = [];
            self.cost = 0;
        end
    
    end
end