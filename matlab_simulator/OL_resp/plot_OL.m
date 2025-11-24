clear
close all
clc

% 
% data_list = ["2025-11-14--21-52-11"];
% for data = data_list
% 
%     curr_data = load(data);
% 
% end

data_name = "2025-11-14--21-52-11.mat";

data = load(data_name);

omegas = data.group.omegas;
iqs = data.group.iqs;
iq_ref = data.group.iq_ref;
time = data.group.time;


[N_exp, t_len] = size(omegas);

omega_0_2 = omegas(:,20);

[~, order] = sort(omega_0_2);


% colors = turbo(N_exp);
% colors = flip(parula(N_exp));
n_colors = 256;
% colors = parula(n_colors);
% colors = flip(parula(n_colors));


% b_channel = zeros(n_colors, 1);
% r_channel = zeros(n_colors, 1);
% g_channel = linspace(0, 1, n_colors)';
start_color_g = [0, 0.5, 0];
end_color_g   = [0.5, 0.8, 0.5];

% Create the gradient for each channel
r_channel = linspace(start_color_g(1), end_color_g(1), n_colors)';
g_channel = linspace(start_color_g(2), end_color_g(2), n_colors)';
b_channel = linspace(start_color_g(3), end_color_g(3), n_colors)';
colors = [r_channel, g_channel, b_channel];




c_min = min(omega_0_2);
c_max = max(omega_0_2);

% color_idxs = round((omegas - c_min) / (c_max - c_min) * (n_colors-1) + 1);
color_scale = linspace(c_min, c_max, size(colors, 1));
line_colors = interp1(color_scale, colors, omega_0_2);


% real exp


data_list = ["2025-11-17--09-21-04_in07", ...
             "2025-11-17--09-31-10_in11", ...
             "2025-11-17--09-34-39_in09", ...
             "2025-11-17--09-38-04_in15", ...
             "2025-11-17--09-40-37_in13", ...
             "2025-11-17--09-43-32_in05"];

omegas_real = [];
iqs_real = [];
iq_refs_real = [];
times_real = [];
for i = 1:length(data_list)

    data = data_list(i);

    curr_data = load(data);
    omegas_real(i,:) = curr_data.group.omega;
    iqs_real(i,:) = curr_data.group.iq;
    iq_refs_real(i,:) = curr_data.group.iq_ref;
    times_real(i,:) = curr_data.group.time - curr_data.group.time(2);

end



omega_0_2_real = omegas_real(:,200);
[~, order_real] = sort(omega_0_2_real);

% colors_real = copper(n_colors);

% b_channel = ones(n_colors, 1);
% g_channel = ones(n_colors, 1);
% r_channel = linspace(0, 1, n_colors)';
start_color_r = [0.5, 0, 0];
end_color_r   = [0.8, 0.5, 0.5];

r_channel = linspace(start_color_r(1), end_color_r(1), n_colors)';
g_channel = linspace(start_color_r(2), end_color_r(2), n_colors)';
b_channel = linspace(start_color_r(3), end_color_r(3), n_colors)';
colors_real = [r_channel, g_channel, b_channel];


c_min = min(omega_0_2_real);
c_max = max(omega_0_2_real);
color_scale_real = linspace(c_min, c_max, size(colors_real, 1));
line_colors_real = interp1(color_scale_real, colors_real, omega_0_2_real);
%%% [2;1;3;4;5;6] order real

line_colors_real = [220,47,2;
                    208, 0, 0;
                    232, 93, 4;
                    244, 140, 6;
                    250, 163, 7;
                    255, 186, 8]/256;




figure('Position',[100,100,500,200])
for i = 1:N_exp
    % subplot(211)
    hold on
    plot(time, omegas(order(i),:), 'Color',line_colors(order(i),:))
    % subplot(212)
    % hold on
    % plot(time, iqs(order(i),:), 'Color',line_colors(order(i),:))
end


for i = 1:length(data_list)
    % subplot(211)
    hold on
    plot(times_real(order_real(i),:), omegas_real(order_real(i),:), 'Color',line_colors_real(order_real(i),:), 'LineWidth',2)
    % subplot(212)
    % hold on
    % plot(times_real(order_real(i),:), iqs_real(order_real(i),:), 'Color',line_colors_real(order_real(i),:), 'LineWidth',2)
end
% subplot(211)
L1 = plot(nan, nan, 'color', [0.25,0.65,0.25]);
L2 = plot(nan, nan, 'color', [244, 140, 6]/256, 'LineWidth',2);
legend([L1, L2], {'Simulated trajectories', 'Real motor trajectories'})
xlim([0,1])
xlabel('Time [s]')
ylabel('Speed [rpm]')
% subplot(212)
% L1 = plot(nan, nan, 'color', [0.25,0.65,0.25]);
% L2 = plot(nan, nan, 'color', [244, 140, 6]/256, 'LineWidth',2);
% Ref = plot(time, iq_ref, ':', 'LineWidth',3, 'Color','k');
% legend([L1, L2, Ref], {'Simulated trajectories', 'Real motor trajectories', 'Current reference'})
% xlim([0,1])

% savefig("../figs_paper/OL_traj.fig")
% saveas(gcf, "../figs_paper/OL_traj.png")
% fig = gcf;
% set(fig, 'PaperPositionMode', 'auto');
% exportgraphics(fig, '../figs_paper/OL_traj.pdf', 'ContentType', 'vector');