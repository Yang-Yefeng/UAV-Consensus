clear; clc;

for index = 0:1:3
    base_dir = ['./pos_consensus-sim-group3/uav_', int2str(index)];
    obs = csvread([base_dir, '/observe.csv'], 1, 0);
    time = obs(:, 1);
    %% x
    figure(3 * index + 1);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, obs(:, 11), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, obs(:, 14) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% y
    figure(3 * index + 2);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, obs(:, 12), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, obs(:, 15) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% z
    figure(3 * index + 3);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, obs(:, 13), 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, obs(:, 16) , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
end
