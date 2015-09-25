classdef RrtStar < handle
    
    properties
        dim
        domain
        tree
        size
        kd_tree
    end
    
    methods
        
        function self = RrtStar(dim, domain)
            %Constructor
            % dimension is dimension of state space
            % domain is an array containing limits of the space in R^n
            self.dim = dim;
            self.size = 0;
            
            if length(domain) == 2 * dim
                self.domain = reshape(domain, [2, length(domain)/2]);
            else
                error('domain size error');
            end
            
            leaf_size = 1;
            distance = @(x, y)norm(x-y, 2);
            self.kd_tree = KDTree(dim, leaf_size, distance);
            
        end
        
        function build_rrt(self, number_nodes, x_init)
            %Grow tree from initial point
            % number_nodes is number of nodes in the tree
            % x_init is the initial point
            
            % Init tree
            root = RrtNode(x_init, nan);
            self.tree = root;
            self.size = self.size + 1;
            self.kd_tree.insert(x_init, root)
            
            for n = 1:1:number_nodes
                % Sample
                x_rand = self.sample();
                % Extend
                self.extend(x_rand);
            end
        end
    end
    
    methods (Access = private)
        
        function x = sample(self)
            %Return a point sample uniformly at random from the domain
            x = self.domain(1, :) + 0.5 * (self.domain(2, :) - self.domain(1, :)) .* rand(1, self.dim);
        end
        
        function [x_nearest, n_nearest] = get_nearest(self, x)
            %Find a node in tree that is nearest to x
            [x_nearest, n_nearest] = self.kd_tree.find_nearest(x);
        end
        
        function [x_new, edge] = steer(self, x_nearest, x_sample)
            %Steer from nearest node to sampled node and return a new node
            x_new = x_sample;
            edge = x_new - x_nearest;
        end
        
        function b = is_obstacle_free(self, x_near, x_new, edge)
            %Return a boolean on whether or not the path from nearest node
            % to new node is obstacle free or not
            b = 1;
        end
        
        function insert(self, parent, x_new, edge)
            child = RrtNode(x_new, parent);
            parent.children = cat(1, parent.children, child);
            parent.edges = cat(1, parent.edges, edge);
            self.kd_tree.insert(x_new, child);
            self.size = self.size + 1;
        end
        
        function extend(self, x_rand)
            %Extend tree towards sampled-point x
            [x_near, n_near] = self.get_nearest(x_rand);
            [x_new, edge] = self.steer(n_near.value, x_rand);
            if self.is_obstacle_free(n_near.value, x_new, edge);
                self.insert(n_near, x_new, edge);
            end
        end
        
    end
end