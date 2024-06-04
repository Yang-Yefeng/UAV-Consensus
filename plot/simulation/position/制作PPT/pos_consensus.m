clear; clc;

for index = 0:1:3
    base_dir = ['./pos_consensus-sim-group3/uav_', int2str(index)];
    ref = csvread([base_dir, '/ref_cmd.csv'], 1, 0);
    state = csvread([base_dir, '/uav_state.csv'], 1, 0);
    time = ref(:, 1);
    %% x
    figure(3 * index + 1);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 2), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 2) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% y
    figure(3 * index + 2);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 3), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 3) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% z
    figure(3 * index + 3);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 4), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 4) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
end
% ref= csvread('./pos_consensus-sim-group1/ref_cmd.csv', 1, 0);
% 
% state_bs_fntsmc = csvread('./pos_bs_fntsmc/uav_state.csv', 1, 0);
% % state_fntsmc_rl_obs = csvread('./pos_bs_fntsmc/uav_state.csv', 1, 0);
% % state_fntsmc_rl = csvread('./pos_bs_fntsmc/uav_state.csv', 1, 0);
% % state_fntsmc_obs = csvread('./FNTSMC-OBS/uav_state.csv', 1, 0);
% 
% time = ref(:, 1);
% pos_ref = ref(:, 2 : 4);
% % psi_ref = ref(:, 7) * 180 / pi;
% 
% %% 得到数据
% pos_bs_fntsmc = state_bs_fntsmc(:, 2 : 4);
% 
% %% Position
% % phi
% figure(1);
% set(gca, 'LooseInset', [0, 0, 0, 0]);
% set(gca, 'FontSize', 10);
% set(gcf, 'unit', 'centimeters', 'position', [25 15 7 5]);
% plot(time, pos_ref(:, 1), 'linewidth', 2, 'color', 'red');
% hold on;
% plot(time, pos_bs_fntsmc(:, 1) , 'linewidth', 2, 'color', 'blue');
% % hold on;
% % plot(time, pos_fntsmc_rl_obs(:, 1), 'linewidth', 1.5, 'color', 'red');
% % hold on;
% % plot(time, pos_fntsmc_rl(:, 1), 'linewidth', 1.5, 'color', '#ffa500');
% % hold on;
% % plot(time, pos_fntsmc_obs(:, 1), 'linewidth', 1.5, 'color', 'green');
% xlim([0, 20]);
% % ylim([-1.5, 2]);
% grid();
% 
% % theta
% figure(2);
% set(gca, 'LooseInset', [0,0,0,0]);
% set(gca, 'FontSize', 10);
% set(gcf, 'unit', 'centimeters', 'position', [25 15 7 5]);
% plot(time, pos_ref(:, 2), 'linewidth', 2, 'color', 'red');
% hold on;
% plot(time, pos_bs_fntsmc(:, 2) , 'linewidth', 2, 'color', 'blue');
% % hold on;
% % plot(time, pos_fntsmc_rl_obs(:, 2), 'linewidth', 1.5, 'color', 'red');
% % hold on;
% % plot(time, pos_fntsmc_rl(:, 2), 'linewidth', 1.5, 'color', '#ffa500');
% % hold on;
% % plot(time, pos_fntsmc_obs(:, 2), 'linewidth', 1.5, 'color', 'green');
% xlim([0, 20]);
% % ylim([-1.5, 2]);
% grid();
% 
% % psi
% figure(3);
% set(gca, 'LooseInset', [0,0,0,0]);
% set(gca, 'FontSize', 10);
% set(gcf, 'unit', 'centimeters', 'position', [25 15 7 5]);
% plot(time, pos_ref(:, 3), 'linewidth', 2, 'color', 'red');
% hold on;
% plot(time, pos_bs_fntsmc(:, 3) , 'linewidth', 2, 'color', 'blue');
% % hold on;
% % plot(time, pos_fntsmc_rl_obs(:, 3), 'linewidth', 1.5, 'color', 'red');
% % hold on;
% % plot(time, pos_fntsmc_rl(:, 3), 'linewidth', 1.5, 'color', '#ffa500');
% % hold on;
% % plot(time, pos_fntsmc_obs(:, 3), 'linewidth', 1.5, 'color', 'green');
% xlim([0, 20]);
% % ylim([0., 1.5]);
% grid();
