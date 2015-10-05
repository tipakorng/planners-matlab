classdef RrtStar < handle
    
    properties
        
        % Planning problem
        problem
        
        % k-d tree
        kd_tree
        
        % Boundary of problem
        domain
        
        % root of RRT*
        tree
        
        % Number of nodes
        tree_size
        
        % Collision-checking resolution
        resolution
        
    end
    
    methods (Access = public)
        
        function obj = RrtStar(planning_problem, resolution)
            % Constructor
            % dimension is dimension of state space
            % domain is an array containing limits of the space in R^n
            obj.problem = planning_problem;
            obj.domain = planning_problem.get_domain();
            obj.tree_size = 0;
            leaf_size = 1;
            obj.kd_tree = KDTree(obj.problem.get_dim(), leaf_size, @(x, y)norm(x - y, 2));
            obj.resolution = resolution;
        end
        
        function build_rrt(obj, number_nodes, x_init)
            % Grow tree from initial point
            % number_nodes is number of nodes in the tree
            % x_init is the initial point
            
            % Init tree
            root = RrtNode(x_init, nan);
            obj.tree = root;
            obj.kd_tree.insert(x_init, root)
            
            for n = 1:1:number_nodes
                % Sample
                x_rand = obj.sample();
                % Extend
                obj.extend(x_rand);
            end
            
        end
        
        function plot(obj)
            % Plot RRT
            obj.plot_edges(obj.tree);
        end
        
    end
    
    methods (Access = private)
        
        function x = sample(obj)
            upper = obj.domain(1, 2:2:end);
            lower = obj.domain(1, 1:2:end-1);
            % Return a point sample uniformly at random from the domain
            x = lower + (upper - lower) .* rand(1, obj.problem.get_dim());
        end
        
        function n_nearest = get_nearest(obj, x)
            % Find a node in tree that is nearest to x
            [~, n_nearest] = obj.kd_tree.find_nearest(x);
        end
        
        function N_near = get_near(obj, x)
            % Return an array of points and their nodes near a point x.
            % A point x' is near a given point x if x' is in a ball with
            % radius k * (log(n) / n)^(1/d) centered at x 
            % (see Karaman 2011).
            domain_size = norm(obj.domain(2:2:end)-obj.domain(1:2:end-1), 2);
            free_space_volume = prod(domain_size);
            ball_volume = pi^(obj.problem.get_dim()/2) / gamma(obj.problem.get_dim()/2 + 1);
            k = 2 * ((1 + 1/obj.problem.get_dim()) * free_space_volume / ball_volume)^(1/obj.problem.get_dim());  % For debugging, actual calculation in paper
            radius = k * (log(obj.tree_size) / obj.tree_size)^(1/obj.problem.get_dim());
            N_near = obj.kd_tree.query_ball(x, radius);
        end
        
        function b = is_collision_free(obj, x_near, edge)
            % Return a boolean on whether or not the path from nearest node
            % to new node is obstacle free or not
            
            num_steps = floor(norm(edge, 2) / obj.resolution);
            x = x_near;
            
            if ~obj.problem.is_obstacle_free(x)
                b = false;
                return
            end
            
            for i = 1 : 1 : num_steps
                x = x + obj.resolution * edge / norm(edge, 2);
                
                if ~obj.problem.is_obstacle_free(x)
                    b = false;
                    return
                end
                
            end
            
            if ~obj.problem.is_obstacle_free(x_near + edge);
                b = false;
                return
            end
            
            b = true;
            
        end
        
        function child = insert(obj, parent, x_new, edge)
            child = RrtNode(x_new, parent);
            child.cost = parent.cost + obj.problem.cost(parent.value, edge);
            if isempty(parent.children)
                parent.children = child;
            else
                parent.children = cat(1, parent.children, [child]);
            end
            
            if isempty(parent.edges)
                parent.edges = edge;
            else
                parent.edges = cat(1, parent.edges, edge);
            end
            obj.kd_tree.insert(x_new, child);
            obj.tree_size = obj.tree_size + 1;
        end
        
        function remove(obj, n)
            % Remove a node from the tree
            p = n.parent;
            index = nan;
            k = 0;
            found = 0;
            
            while ~found
                k = k + 1;
                
                if p.children(k) == n
                    index = k;
                    found = 1;
                end
                
            end
            
            p.children(index) = [];
            p.edges(index, :) = [];
            n.parent = [];
        end
        
        function rewire(n_near, n_new)
            obj.remove(n_near);
            n_near.parent = n_new;
            n_near.cost = n_new.cost + obj.problem.cost;
        end
        
        function extend(obj, x_rand)
            % Extend tree towards sampled-point x
            n_nearest = obj.get_nearest(x_rand);
            [x_new, edge_new] = obj.problem.steer(n_nearest.value, x_rand);
            
            % If collision free, try to connect from all near nodes and rewire tree
            if obj.is_collision_free(n_nearest.value, edge_new);
                N_near = obj.get_near(x_new);
                n_min = n_nearest;
                edge_min = edge_new;
                c_min = n_nearest.cost + obj.problem.cost(n_nearest.value, edge_new);
                
                % Find node from near nodes that yield minimum cost to x_new
                for k = 1:1:size(N_near, 1)
                    n_near = N_near(k);
                    [~, edge] = obj.problem.steer(n_near.value, x_new);
                    
                    if obj.is_collision_free(n_near.value, edge) && n_near.cost + obj.problem.cost(n_near.value, edge) < c_min
                        n_min = n_near;
                        edge_min = edge;
                        c_min = n_near.cost + obj.problem.cost(n_near.value, edge);
                    end
                    
                end
                
                n_new = obj.insert(n_min, x_new, edge_min);
                
                % Rewire tree locally
                for k = 1:1:size(N_near, 1)
                    n_near = N_near(k);
                    [~, edge] = obj.problem.steer(n_new.value, n_near.value);
                    
                    % If there is an edge from x_new to x_near that has lower cost,
                    % change parent of x_near to x_new.
                    if obj.is_collision_free(n_new.value, edge) && n_new.cost + obj.problem.cost(n_new.value, edge) < n_near.cost
                        obj.remove(n_near);
                        n_near.parent = n_new;
                        n_new.children = cat(1, n_new.children, n_near);
                        n_new.edges = cat(1, n_new.edges, edge);
                        n_near.cost = n_new.cost + obj.problem.cost(x_new, edge);
                    end
                    
                end
                
            end
            
        end
        
        function plot_edges(obj, node)
            % Plot edges of a given node and return child
            for n = 1:size(node.edges, 1)
                plot([node.value(1), node.children(n).value(1)], [node.value(2), node.children(n).value(2)]);
                obj.plot_edges(node.children(n));
            end
        end
        
    end
    
end
