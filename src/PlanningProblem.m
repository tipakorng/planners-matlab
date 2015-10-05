classdef PlanningProblem < handle
    
    properties (Access = protected)
        
        dim
        
        domain
        
    end
    
    methods (Access = public)
        
        function dim = get_dim(obj)
            dim = obj.dim;
        end
        
        function domain = get_domain(obj)
            domain = obj.domain;
        end
        
    end
    
    methods (Access = public, Abstract)
        
        d = cost(obj, vertex, edge)
        
        b = is_obstacle_free(obj, vertex, edge)
        
        [x_new, edge] = steer(obj, x_nearest, x_sample)
        
    end
end