classdef Rrt < handle
    
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
        
        function obj = Rrt(planning_problem, resolution)
        %
        % obj = Rrt(planning_problem, resolution)
        %
        % Constructor
        %
        % Inputs: 
        % planning_problem is a PlanningProblem object.
        % resolution is the collision-checking resolution.
        %
        % Output:
        % obj is a concrete object of this class.
        %
            obj.problem = planning_problem;
            obj.domain = planning_problem.get_domain();
            obj.tree_size = 0;
            leaf_size = 1;
            obj.kd_tree = KDTree(obj.problem.get_dim(), leaf_size, @(x, y)norm(x - y, 2));
            obj.resolution = resolution;
        end
        
        function build_rrt(obj, number_nodes, x_init)
        %
        % build_rrt(number_nodes, x_init)
        %
        % Grow tree from an initial point.
        %
        % Inputs:
        % number_nodes is the number of nodes in the tree.
        % x_init is the initial point.
        %
            
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
        %
        % plot()
        %
        % Plot the tree.
        %
            obj.plot_edges(obj.tree);
        end
        
    end
    
    methods (Access = protected)
        
        function x = sample(obj)
        %
        % x = sample()
        %
        % Sample a point in the planning domain uniformly.
        %
        % Output:
        % x is the sampled point.
        %
            upper = obj.domain(1, 2:2:end);
            lower = obj.domain(1, 1:2:end-1);
            % Return a point sample uniformly at random from the domain
            x = lower + (upper - lower) .* rand(1, obj.problem.get_dim());
        end
        
        function n_nearest = get_nearest(obj, x)
        %
        % n_nearest = get_nearest(x)
        %
        % Find a node in tree that is nearest to x.
        %
        % Input:
        % x is a querry point.
        %
        % Output:
        % n_nearest is the nearest node in the k-d tree.
        %
            [~, n_nearest] = obj.kd_tree.find_nearest(x);
        end
        
        function child = insert(obj, parent, x, edge)
        %
        % child = insert(parent, x_new, edge)
        %
        % Insert a new point and an associated node to the tree.
        %
        % Inputs:
        % parent is an RRT node that the new point will be assigned to as its
        %  child.
        % x is the new point.
        % edge is the new edge.
        %
        % Output:
        % child is the new RRT node.
        %
            
            % Create a new RRT node
            child = RrtNode(x, parent);
            child.cost = parent.cost + obj.problem.cost(parent.vertex, edge);
            
            % Assign child to parent
            if isempty(parent.children)
                parent.children = child;
            else
                parent.children = cat(1, parent.children, child);
            end
            
            % Assign edge to parent
            if isempty(parent.edges)
                parent.edges = edge;
            else
                parent.edges = cat(1, parent.edges, edge);
            end
            
            % Insert new point to k-d tree
            obj.kd_tree.insert(x, child);
            % Increment tree size
            obj.tree_size = obj.tree_size + 1;
        end
        
        function b = is_collision_free(obj, x, edge)
        %
        % b = is_collision_free(x_near, edge)
        %
        % Return a boolean on whether or not the path from nearest node
        % to new node is obstacle free or not.
        %
        % Inputs:
        % x is a point.
        % edge is a path from the point x.
        %
        % Output:
        % b is true is path is collision free and is false otherwise.
        %
            
            num_steps = floor(norm(edge, 2) / obj.resolution);
            x_middle = x;
            
            if ~obj.problem.is_obstacle_free(x_middle)
                b = false;
                return
            end
            
            for i = 1 : 1 : num_steps
                x_middle = x_middle + obj.resolution * edge / norm(edge, 2);
                
                if ~obj.problem.is_obstacle_free(x_middle)
                    b = false;
                    return
                end
                
            end
            
            if ~obj.problem.is_obstacle_free(x + edge);
                b = false;
                return
            end
            
            b = true;
            
        end
        
        function extend(obj, x_rand)
        %
        % extend(x_rand)
        %
        % Extend tree towards sampled-point x.
        %
        % Input:
        % x_rand is the point the tree will extend towards.
        %
            
            % Get nearest point
            n_nearest = obj.get_nearest(x_rand);
            % Generate a new point
            [x_new, edge_new] = obj.problem.steer(n_nearest.vertex, x_rand);
            
            % If collision free, add x_new and edge_new to tree
            if obj.is_collision_free(n_nearest.vertex, edge_new);
                obj.insert(n_nearest, x_new, edge_new);
            end
            
        end
        
        function plot_edges(obj, node)  % TODO: Make plotting dimension selectable.
        %
        % plot_edges(node)
        %
        % Plot edges of a given node and recursively.
        % Only support 2-dim at the moment.
        %
        % Input:
        % node is an RRT node.
        %
            for n = 1:size(node.edges, 1)
                plot([node.vertex(1), node.children(n).vertex(1)], [node.vertex(2), node.children(n).vertex(2)]);
                obj.plot_edges(node.children(n));
            end
        end
        
    end
    
end
