clear; clc;

%% 得到数据
obs_data = csvread('./att_bs_fntsmc/observe.csv', 1, 0);
time = obs_data(:, 1);
obs_real = obs_data(:, 2 : 4);
obs_esti = obs_data(:, 5 : 7);

%% phi
figure(1);
set(gca, 'LooseInset', [0, 0, 0, 0]);
set(gcf, 'unit', 'centimeters', 'position', [25 15 10 3]);
plot(time, obs_real(:, 1), 'linewidth', 2, 'color', 'red');
hold on;
plot(time, obs_esti(:, 1) , 'linewidth', 2, 'color', 'blue');
xlim([0, 20]);
% ylim([-1.5, 2]);
set(gca, 'FontSize', 10);
grid();

%% theta
figure(2);
set(gca, 'LooseInset', [0, 0, 0, 0]);
set(gcf, 'unit', 'centimeters', 'position', [25 15 10 3]);
plot(time, obs_real(:, 2), 'linewidth', 2, 'color', 'red');
hold on;
plot(time, obs_esti(:, 2) , 'linewidth', 2, 'color', 'blue');
xlim([0, 20]);
% ylim([-1.5, 2]);
set(gca, 'FontSize', 10);
grid();

%% psi
figure(3);
set(gca, 'LooseInset', [0, 0, 0, 0]);
set(gcf, 'unit', 'centimeters', 'position', [25 15 10 3]);
plot(time, obs_real(:, 3), 'linewidth', 2, 'color', 'red');
hold on;
plot(time, obs_esti(:, 3) , 'linewidth', 2, 'color', 'blue');
xlim([0, 20]);
% ylim([-1.5, 2]);
set(gca, 'FontSize', 10);
grid();