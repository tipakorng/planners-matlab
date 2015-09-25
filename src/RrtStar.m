classdef RrtStar
    
    properties
        dim
        domain
        tree
    end
    
    methods
        
        function obj = RrtStar(dim, domain)
            %Constructor
            % dimension is dimension of state space
            % domain is an array containing limits of the space in R^n
            obj.dim = dim;
            
            if sum(size(domain)) == 2 * dim
                obj.domain = domain;
            else
                error('domain size error');
            end
            
        end
        
        function x = sample()
            %Return a point sample uniformly at random from the domain
            x = obj.domain(:, 0) + 0.5 * (obj.domain(:, 1) - obj.domain(:, 0)) * rand(obj.dim, 1);
        end
        
        function x_nearest = get_nearest(tree, x)
            %Find a node in tree that is nearest to x
            
        end
        
        function x_new = steer(x_nearest, x_sample)
            %Steer from nearest node to sampled node and return a new node
        end
        
        function obstacle_free = is_obstacle_free()
            %Return a boolean on whether or not the path from nearest node
            % to new node is obstacle free or not
        end
        
        function [node, edge] = extend(tree, x)
            %Extend tree towards sampled-point x
        end
        
        function grow_tree(number_nodes, root)
            %grow tree from root node
            % number_nodes is number of nodes in the tree
            % root is the root node
            
            % Init tree
            obj.tree = Node(root);
            
            for n = 1:1:number_nodes
                % Sample
                % Extend
            end
        end
    end
end