%%
clc

%% Build RRT
dim = 2;
domain = [0, 1, 0, 1];
N = 100;
x_init = [0.5, 0.5];
rrt = RrtStar(dim, domain);
rrt.build_rrt(N, x_init);