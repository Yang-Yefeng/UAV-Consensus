clear; clc;

%% 得到数据
pos_param = csvread('./opt_param/opt_param_pos.csv', 1, 0);
att_param = csvread('./opt_param/opt_param_att.csv', 1, 0);
time = 0.01: 0.01: 20;

%% define color
Orange = [0, 165, 255] / 255;
LightPink = [193, 182, 255] / 255;
Purple = [240, 32, 160] / 255;
DarkGreen = [0, 100, 0] / 255;
Magenta = [255, 0, 255] / 255;

%% x
figure(1);
set(gca, 'LooseInset', [0, 0, 0, 0]);
set(gcf, 'unit', 'centimeters', 'position', [25 15 7 6]);
plot(time, pos_param(:, 1), 'linewidth', 1.5, 'color', 'red');
hold on;
plot(time, pos_param(:, 2) , 'linewidth', 1.5, 'color', 'blue');
hold on;
plot(time, pos_param(:, 3) , 'linewidth', 1.5, 'color', 'green');
hold on;
plot(time, pos_param(:, 4) , 'linewidth', 1.5, 'color', Orange);
hold on;
plot(time, pos_param(:, 5) , 'linewidth', 1.5, 'color', LightPink);
hold on;
plot(time, pos_param(:, 6) , 'linewidth', 1.5, 'color', Purple);
hold on;
plot(time, pos_param(:, 7) , 'linewidth', 1.5, 'color', DarkGreen);
hold on;
plot(time, pos_param(:, 8) , 'linewidth', 1.5, 'color', Magenta);
xlim([0, 20]);
ylim([0.5, 2]);
set(gca, 'FontSize', 10);
grid();

figure(2);
set(gca, 'LooseInset', [0, 0, 0, 0]);
set(gcf, 'unit', 'centimeters', 'position', [25 15 7 6]);
plot(time, att_param(:, 1), 'linewidth', 1.5, 'color', 'red');
hold on;
plot(time, att_param(:, 2) , 'linewidth', 1.5, 'color', 'blue');
hold on;
plot(time, att_param(:, 3) , 'linewidth', 1.5, 'color', 'green');
hold on;
plot(time, att_param(:, 4) , 'linewidth', 1.5, 'color', Orange);
hold on;
plot(time, att_param(:, 5) , 'linewidth', 1.5, 'color', LightPink);
hold on;
plot(time, att_param(:, 6) , 'linewidth', 1.5, 'color', Purple);
hold on;
plot(time, att_param(:, 7) , 'linewidth', 1.5, 'color', DarkGreen);
hold on;
plot(time, att_param(:, 8) , 'linewidth', 1.5, 'color', Magenta);
xlim([0, 20]);
% ylim([-1.5, 2]);
set(gca, 'FontSize', 10);
grid();
