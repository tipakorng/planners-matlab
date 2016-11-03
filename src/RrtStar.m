classdef RrtStar < Rrt
    
    methods (Access = public)
        
        function obj = RrtStar(planning_problem, resolution)
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
            obj = obj@Rrt(planning_problem, resolution);
            
        end
        
    end
    
    methods (Access = protected)
        
        function N_near = get_near(obj, x)
        %
        % N_near = get_near(x)
        %
        % Return an array of points and their nodes near a point x.
        % A point x' is near a given point x if x' is in a ball with
        % radius k * (log(n) / n)^(1/d) centered at x 
        % (see Karaman 2011).
        %
        % Input:
        % x is the querry point.
        %
        % Output:
        % N_near is a list of nodes that are near enough.
        %
            domain_size = norm(obj.domain(2:2:end)-obj.domain(1:2:end-1), 2);
            free_space_volume = prod(domain_size);
            ball_volume = pi^(obj.problem.get_dim()/2) / gamma(obj.problem.get_dim()/2 + 1);
            k = 2 * ((1 + 1/obj.problem.get_dim()) * free_space_volume / ball_volume)^(1/obj.problem.get_dim());  % For debugging, actual calculation in paper
            radius = k * (log(obj.tree_size) / obj.tree_size)^(1/obj.problem.get_dim());
            N_near = obj.kd_tree.query_ball(x, radius);
        end
        
        function extend(obj, x_rand)
        %
        % extend(x_rand)
        %
        % Extend tree towards sampled-point x_rand.
        %
        % Input:
        % x_rand is a point the tree will extend towards.
        %
            n_nearest = obj.get_nearest(x_rand);
            [x_new, edge_new] = obj.problem.steer(n_nearest.vertex, x_rand);
            
            % If collision free, try to connect from all near nodes and rewire tree
            if obj.is_collision_free(n_nearest.vertex, edge_new);
                N_near = obj.get_near(x_new);
                n_min = n_nearest;
                edge_min = edge_new;
                c_min = n_nearest.cost + obj.problem.cost(n_nearest.vertex, edge_new);
                
                % Find node from near nodes that yield minimum cost to x_new
                for k = 1:1:size(N_near, 1)
                    n_near = N_near(k);
                    [~, edge] = obj.problem.steer(n_near.vertex, x_new);
                    
                    if obj.is_collision_free(n_near.vertex, edge) && n_near.cost + obj.problem.cost(n_near.vertex, edge) < c_min
                        n_min = n_near;
                        edge_min = edge;
                        c_min = n_near.cost + obj.problem.cost(n_near.vertex, edge);
                    end
                    
                end
                
                n_new = obj.insert(n_min, x_new, edge_min);
                
                % Rewire tree locally
                for k = 1:1:size(N_near, 1)
                    n_near = N_near(k);
                    [~, edge] = obj.problem.steer(n_new.vertex, n_near.vertex);
                    
                    % If there is an edge from x_new to x_near that has lower cost,
                    % change parent of x_near to x_new.
                    if obj.is_collision_free(n_new.vertex, edge) && n_new.cost + obj.problem.cost(n_new.vertex, edge) < n_near.cost
                        obj.remove(n_near);
                        n_near.parent = n_new;
                        n_new.children = cat(1, n_new.children, n_near);
                        n_new.edges = cat(1, n_new.edges, edge);
                        n_near.cost = n_new.cost + obj.problem.cost(x_new, edge);
                    end
                    
                end
                
            end
            
        end
        
    end
    
    methods (Static)
        
        function remove(n)
        %
        % remove(n)
        %
        % Remove a node from the tree.
        %
        % Input:
        % n is the node to be removed.
        %
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
        %
        % rewire(n_near, n_new)
        %
        % Rewire the tree
        %
        % Inputs:
        % n_near is a node near n_new
        % n_new is the new node
        %
            obj.remove(n_near);
            n_near.parent = n_new;
            n_near.cost = n_new.cost + obj.problem.cost;
        end
    end
    
end
