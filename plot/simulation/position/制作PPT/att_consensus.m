clear; clc;

for index = 0:1:3
    base_dir = ['./pos_consensus-sim-group3/uav_', int2str(index)];
    ref = csvread([base_dir, '/ref_cmd.csv'], 1, 0);
    state = csvread([base_dir, '/uav_state.csv'], 1, 0);
    time = ref(:, 1);
    %% phi
    figure(3 * index + 1);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 8)*180/pi, 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 8)*180/pi , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% theta
    figure(3 * index + 2);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 9)*180/pi, 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 9)*180/pi , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    % ylim([-1.5, 2]);
    grid();
    %% psi
    figure(3 * index + 3);
    set(gca, 'LooseInset', [0, 0, 0, 0]);
    set(gca, 'FontSize', 10);
    set(gcf, 'unit', 'centimeters', 'position', [25 15 7 4]);
    plot(time, ref(:, 10)*180/pi, 'linewidth', 2, 'color', 'red');
    hold on;
    plot(time, state(:, 10)*180/pi , 'linewidth', 2, 'color', 'blue');
    xlim([0, 20]);
    ylim([-100, 100]);
    grid();
end
