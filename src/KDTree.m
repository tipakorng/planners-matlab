classdef KDTree < handle
    
    properties
        dim
        leaf_size
        tree
        depth
        size
        distance
        box
    end
    
    methods
        
        function self = KDTree(dim, leaf_size, distance)
            %Constructor
            % dim is dimension of the data.
            % leaf_size is the maximum number of point in a leaf node.
            self.dim = dim;
            self.leaf_size = leaf_size;
            self.tree = nan;
            self.depth = 0;
            self.size = 0;
            self.distance = distance;
            self.box = nan(1, 2*self.dim);
        end
    
        function build_tree(self, keys, values)
            % Build k-d tree from array of points
            % P is an array of data points where each row represents a
            % point in R^n.

            % Calculate bounding box
            b = zeros(1, 2*self.dim);
            for n = 1:1:self.dim
                b(2*n-1) = min(keys(:, n));
                b(2*n) = max(keys(:, n));
            end
            
            % Build k-d tree
            self.tree = self.build_kd_tree(keys, values, b, nan, 0);
        end
        
        function [key, value] = find_nearest(self, query)
            %Find nearest point in the tree to a given point.
            % q is a given point that may or may not be on the tree.
            % p is the nearest point to q.
            [key, value] = self.nns(query, self.tree, nan, nan, inf);
        end
        
        function insert(self, key, value)
            %Insert a new point into tree.
            % p is a point to be inserted into the tree.
            self.tree = self.insert_node(key, value, self.tree, 0);
            self.update_bounding_box(key);
        end
        
        function value = query_ball(self, q, radius)
            % Query all nodes that are within radius r from point q
            value = self.traverse_checking(self.tree, self.box, q, radius);
        end
        
    end
    
    methods (Access = private)
        
        function b = get_bounding_box(self, P)
            %Calculate bounding box of a set of points P
            b = zeros(1, 2*self.dim);
            for n = 1:1:self.dim
                b(2*n-1) = min(P(:, n));
                b(2*n) = max(P(:, n));
            end
        end
        
        function update_bounding_box(self, p)
            %Update bounding box when a new point p is added
            self.box(1:2:end-1) = min(p, self.box(1:2:end-1));
            self.box(2:2:end) = max(p, self.box(2:2:end));
        end
        
        function [box_left, box_right] = split_box(self, box, split_axis, split_value)
            box_left = box;
            box_right = box;
            box_left(2*split_axis) = split_value;
            box_right(2*split_axis-1) = split_value;
        end
        
        function [k, s, keys_left, keys_right, values_left, values_right, b_left, b_right] = split(self, keys, values, b)
            %Find the splitting point and dimension given data points and
            % bounding box.
            % P is an array of data points.
            % b is a bounding box containing P.
            % k is the axis of P used for splitting.
            % s is the splitting point along k.
            % b_lo is the bounding box containing points in P that are
            % below splitting value.
            % b_hi is the bounding_box containing points in P that are
            % above splitting value.
            
            % Find longest dimension to split
            dim_max = 0;
            k = 0;
            
            for n = 1:1:self.dim
                dim = b(2*n) - b(2*n-1);
                
                if dim > dim_max
                    dim_max = dim;
                    k = n;
                end
                
            end
            
            % Set split value and bounding boxes
            [keys, index] = sortrows(keys, k);
            values = values(index);
            n = ceil(size(keys, 1) / 2);
            s = keys(n, k);
            b_left = b;
            b_right = b;
            b_left(2*k) = s;
            b_right(2*k-1) = s;
            
            % Assign points to new boxes
            keys_left = keys(1:n, :);
            keys_right = keys(n+1:end, :);
            values_left = values(1:n);
            values_right = values(n+1:end);
            
        end
        
        function node = build_kd_tree(self, keys, values, box, parent, depth)
            %Build k-d tree from an array of points and a bounding box.
            % P is an array of data points where each point is a row vector
            % of length n.
            % bb is a bounding box containing P.
            % parent is the parent of this node.
            % depth is depth of the current node.
            
            % If the number of points in P is less than some value, turn
            % this into a leaf.
            
            if depth > self.depth
                self.depth = depth;
            end
            
            if size(keys, 1) <= self.leaf_size
                node = KDNode(keys, values, parent);
            else
                [k, s, keys_left, keys_right, values_left, values_right, box_left, box_right] = self.split(keys, values, box);
                self.size = self.size + 1;
                node = KDNode(nan, nan, parent);
                node.split_axis = k;
                node.split_value = s;
                node.left = self.build_kd_tree(keys_left, values_left, box_left, node, depth+1);
                node.right = self.build_kd_tree(keys_right, values_right, box_right, node, depth+1);
            end
        end
        
        function [key, value, distance_best] = leaf_nns(self, query, node)
            %Search leaf for point closest to q
            
            distance_best = inf;
            key = nan;
            value = nan;
            
            for k = 1:1:size(node.keys, 1)
                d = self.distance(query, node.keys(k, :));

                if d < distance_best
                    distance_best = d;
                    key = node.keys(k, :);
                    value = node.values(k);
                end
                    
            end
                
        end
        
        function [key, value] = nns(self, query, node, key, value, distance_best)
            %Nearest neighbor search
            
            % If at leaf node
            if isnan(node.split_axis)
                [key_min, value_min, d_min] = self.leaf_nns(query, node);
                
                if d_min < distance_best
                    key = key_min;
                    value = value_min;
                end
            
            % If not at leaf node
            else
                
                % If candidate point and its distance have not been
                % determined yet
                if distance_best == inf
                    
                    if query(node.split_axis) <= node.split_value
                        [key, value] = self.nns(query, node.left, key, value, distance_best);
                        distance_best = self.distance(key, query);
                        
                        if query(node.split_axis) + distance_best > node.split_value
                            [key, value] = self.nns(query, node.right, key, value, distance_best);
                        end
                        
                    else
                        [key, value] = self.nns(query, node.right, key, value, distance_best);
                        distance_best = self.distance(key, query);
                        
                        if query(node.split_axis) - distance_best <= node.split_value
                            [key, value] = self.nns(query, node.left, key, value, distance_best);
                        end
                        
                    end
                
                % If candidate point is determined
                else
                        
                    if query(node.split_axis) - distance_best <= node.split_value
                        [key, value] = self.nns(query, node.left, key, value, distance_best);
                        distance_best = self.distance(key, query);
                    end
                    
                    if query(node.split_axis) + distance_best > node.split_value
                        [key, value] = self.nns(query, node.right, key, value, distance_best);
                        distance_best = self.distance(key, query);
                    end
                    
                end
            end
            
        end
        
        function node = insert_node(self, key, value, node, depth)
            % Recursively traverse the tree and insert a new point
            if ~isa(node, 'KDNode')
                node = KDNode(key, value, nan);
            elseif isnan(node.split_axis)
                node.keys = cat(1, node.keys, key);
                node.values = cat(2, node.values, value);
                if size(node.keys, 1) >= self.leaf_size
                    box = self.get_bounding_box(node.keys);
                    node = build_kd_tree(self, node.keys, node.values, box, node.parent, depth);
                end
            elseif key(node.split_axis) <= node.split_value
                node.left = self.insert_node(key, value, node.left, depth+1);
            else
                node.right = self.insert_node(key, value, node.right, depth+1);
            end
        end
        
        function d = min_distance_point(self, b, q)
            %Calculate minimum distance from box to point
            d = self.distance(zeros(size(q)), max(zeros(size(q)), max(b(1:2:end-1)-q, q-b(2:2:end))));
        end
        
        function d = max_distance_point(self, b, q)
            %Calculate maximum distance from box to point
            d = self.distance(zeros(size(q)), max(b(2:2:end)-q, q-b(1:2:end-1)));
        end
            
        function v = traverse_checking(self, node, box, query, radius)
            %Implementation of traverse_checking from scipy.spatial.
            % This is used in query_ball method.
            
            if self.min_distance_point(box, query) > radius
                v = [];
            elseif self.max_distance_point(box, query) < radius
                v = self.traverse_no_checking(node);
            elseif isnan(node.split_axis)
                v = [];
                for k = 1:1:size(node.keys, 1)
                    if self.distance(node.keys(k), query) < radius
                        v = cat(1, v, node.values(k));
                    end
                end
            else
                [box_left, box_right] = self.split_box(box, node.split_axis, node.split_value);
                v = [self.traverse_checking(node.left, box_left, query, radius); self.traverse_checking(node.right, box_right, query, radius)];
            end

        end

        function v = traverse_no_checking(self, node)
            %Implementation of traverse_no_checking from scipy.spatial.
            % This is used in query_ball method.
            
            if isnan(node.split_axis)
                v = node.values;
            else
                v = [self.traverse_no_checking(node.left); self.traverse_no_checking(node.right)];
            end
            
        end
        
    end
    
end