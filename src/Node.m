classdef Node
    properties
        parent
        children
    end
    
    methods
        function obj = Node(parent, children)
            obj.parent = parent;
            obj.children = children;
        end
    
    end
end