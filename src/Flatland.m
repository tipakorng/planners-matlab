classdef Flatland < PlanningProblem
    
    properties (Access = private)
        
        num_obstacles
        
        obstacles
        
    end
    
    methods (Access = public)
        
        function obj = Flatland(domain, obstacles)
            obj.dim = 2;
            obj.domain = domain;
            obj.num_obstacles = size(obstacles, 1);
            obj.obstacles = obstacles;
        end
        
        
        function d = cost(obj, vertex, edge)
            d = norm(edge, 2);
        end
        
        function b = is_obstacle_free(obj, vertex)
            
            for i = 1 : 1 : obj.num_obstacles
                obstacle = obj.obstacles(i, :);
                
                if vertex(1) >= obstacle(1) && vertex(1) <= obstacle(2) && vertex(2) >= obstacle(3) && vertex(2) <= obstacle(4)
                    b = false;
                    return
                end
                
            end
            
            b = true;
            
        end
        
        function [x_new, edge] = steer(obj, x_nearest, x_sample)
            edge = (x_sample - x_nearest);
            x_new = x_nearest + edge;
        end
        
        function plot(obj)
            
            for i = 1 : 1 : obj.num_obstacles
                obstacle = obj.obstacles(i, :);
                X = [obstacle(1), obstacle(2), obstacle(2), obstacle(1)];
                Y = [obstacle(3), obstacle(3), obstacle(4), obstacle(4)];
                patch(X, Y, 'red');
            end
        end
    end
end