clc;clear;

%% attitude
data_fntsmc_no_obs = csvread('comparative_cost_surface/att_cost_surface_fntsmc_no_obs.csv', 1, 0);
data_fntsmc_obs = csvread('comparative_cost_surface/att_cost_surface_fntsmc_obs.csv', 1, 0);
data_rl_no_obs = csvread('comparative_cost_surface/att_cost_surface_rl_no_obs.csv', 1, 0);
data_rl_obs = csvread('comparative_cost_surface/att_cost_surface_rl_obs.csv', 1, 0);


A = data_fntsmc_no_obs(:, 1);
T = data_fntsmc_no_obs(:, 2);

%% reward
r_fntsmc_no_obs = data_fntsmc_no_obs(:, 3);
r_fntsmc_obs = data_fntsmc_obs(:, 3);
r_rl_no_obs = data_rl_no_obs(:, 3);
r_rl_obs = data_rl_obs(:, 3);


%% 
figure()
set(gca, 'LooseInset', [0.01, 0.01, 0.01, 0.01]);
[x, y] = meshgrid(linspace(min(A), max(A), 50), linspace(min(T), max(T), 50));

%% plot FNTSMC-NO-OBS
z1 = griddata(A, T, r_fntsmc_no_obs, x, y);
mesh(x, y, z1, 'facecolor', 'b', 'EdgeColor', 'none'); hold on;

%% plot FNTSMC-OBS
z2 = griddata(A, T, r_fntsmc_obs, x, y);
mesh(x, y, z2, 'facecolor', 'r', 'EdgeColor', 'none'); hold on;

%% plot RL-NO-OBS
z3 = griddata(A, T, r_rl_no_obs, x, y);
mesh(x, y, z3, 'facecolor', 'g', 'EdgeColor', 'none'); hold on;

%% plot RL-OBS
z4 = griddata(A, T, r_rl_obs, x, y);
mesh(x, y, z4, 'facecolor', 'cyan', 'EdgeColor', 'none'); hold on;

legend('FNTSMC-NO-OBS', 'FNTSMC-OBS', 'RL-NO-OBS', 'RL-OBS');
title('position');
grid on;
