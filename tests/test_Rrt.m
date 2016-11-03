%%
clc

%% Setup problem

domain = [-1, 1, -1, 1];
obstacles = [[0.4, 0.6, -0.2, 0.2]; [-0.1, 0.1, 0.3, 0.5]];
flatland = Flatland(domain, obstacles);

%% Build tree
N = 500;
resolution = 0.01;
x_init = [0, 0];
rrt = Rrt(flatland, resolution);
disp('build tree...');
rrt.build_rrt(N, x_init);
disp('finish building tree');

%% Plot result
figure
hold on;
rrt.plot()
flatland.plot()
hold off;