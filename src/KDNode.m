classdef KDNode < handle
    
    properties
        keys
        values
        parent
        split_value
        split_axis
        left
        right
    end
    
    methods
        function self = KDNode(keys, values, parent)
            self.keys = keys;
            self.values = values;
            self.parent = parent;
            self.split_axis = nan;
            self.split_value = nan;
            self.left = nan;
            self.right = nan;
        end
    end
    
end