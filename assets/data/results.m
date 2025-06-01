clc; close all; clear;

%% load data
data_file = './data.csv';
data_table = readtable(data_file);
data_array = table2array(data_table);

% data parsing
vec_size = [ ...
    1, ...  % time
    2, ...  % p_des
    2, ...  % p_cal
    2];     % tau_des
start_idx = [1, ];

for i = 1:length(vec_size) - 1
    start_idx(i+1) = start_idx(i) + vec_size(i);
end

data_cell = cell(1, length(vec_size));

for i = 1:length(vec_size)
    data_cell{i} = data_array(:, start_idx(i):start_idx(i) + vec_size(i) - 1);
end

t = data_cell{1};
p_des = data_cell{2};
p_cal = data_cell{3};
tau_des = data_cell{4};

t_start = t(1);
t_end = t(end);

lw = 1.2;
font = 'Times New Roman';
fs_lgd = 14;
fs_lbl = 22;
fs_axis = 14;
fs_title = 20;

%% Plotting
% Reference Tracking
% close all;
figure();
fig_track = tiledlayout(2, 1, "Padding", "compact", "TileSpacing", "compact");

nexttile;
hold on;
grid;
plot(t, p_des(:,1), 'LineStyle', '--', 'LineWidth', 1.5 * lw, 'Color', 'k');
plot(t, p_cal(:,1), 'LineStyle', '-', 'LineWidth', lw, 'Color', 'r');
xlim([t_start, t_end]);
ylim([1 2]);
legend('Reference', 'Measurement', 'FontName', font, 'FontSize', fs_lgd, 'interpreter', 'latex', ...
    'location', 'best');

nexttile;
hold on;
grid;
plot(t, p_des(:,2), 'LineStyle', '--', 'LineWidth', 1.5 * lw, 'Color', 'k');
plot(t, p_cal(:,2), 'LineStyle', '-', 'LineWidth', lw, 'Color', 'r');
xlim([t_start, t_end]);
ylim([-0.15 0.15]);
title(fig_track, 'Position Tracking $[m]$', 'FontSize', fs_title, 'FontName', font, 'Interpreter', 'latex');
xlabel(fig_track, 'Time (sec)', 'Interpreter', 'latex', 'FontSize', fs_lbl, 'FontName', font);
