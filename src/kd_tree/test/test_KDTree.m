%%
clear
clc

%% Build k-d tree
distance = @(x, y)norm(x-y, 2);
dim = 2;
num_points = 100;
kd_tree = KDTree(2, 1, distance);
keys = rand(num_points, dim);
values = zeros(size(keys, 1), 1);
kd_tree.build_tree(keys, values);
kd_tree.size
kd_tree.depth

%% Test find nearest N times
N = 100;
num_successes = 0;

for n = 1:1:N
    % Generate q query point
    query = rand(1, 2);
    % Find nearest points by using nearest neighbor in k-d tree
    [p_nearest, v_nearest] = kd_tree.find_nearest(query);
    % Find nearest point using brute force
    d_min = inf;
    p_compare = nan;

    for k = 1:1:size(keys, 1)

        d = distance(query, keys(k, :));

        if d < d_min
            d_min = d;
            p_compare = keys(k, :);
        end
    end
    
    % If the two nearest points match, increment counter
    if p_compare == p_nearest
        num_successes = num_successes + 1;
    end
end

% 
disp('Number of successes = ');
disp(num_successes);

%% Test point insertion
distance = @(x, y)norm(x-y, 2);
dim = 2;
num_points = 10;
kd_tree = KDTree(2, 1, distance);
num_successes = 0;

for n = 1:1:num_points
    new_key = rand(1, 2);
    new_value = new_key(1, 1);
    kd_tree.insert(new_key, new_value);
    [key, value] = kd_tree.find_nearest(new_key);
    
    if (isequal(key, new_key)) && (isequal(value, new_value))
        num_successes = num_successes + 1;
    end
end

disp('Number of successes = ');
disp(num_successes);