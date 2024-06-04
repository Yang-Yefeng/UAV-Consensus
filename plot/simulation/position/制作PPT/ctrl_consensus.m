clear; clc;

for index = 0:1:3
    base_dir = ['./pos_consensus-sim-group3/uav_', int2str(index)];
    ctrl = csvread([base_dir, '/control.csv'], 1, 0);
    time = ctrl(:, 1);
    %% throttle
    figure(4 * index + 1);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ctrl(:, 2), 'linewidth', 2, 'color', 'red');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% Tx
    figure(4 * index + 2);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ctrl(:, 3), 'linewidth', 2, 'color', 'red');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% Ty
    figure(4 * index + 3);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ctrl(:, 4), 'linewidth', 2, 'color', 'red');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% Tz
    figure(4 * index + 4);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ctrl(:, 5), 'linewidth', 2, 'color', 'red');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
end
