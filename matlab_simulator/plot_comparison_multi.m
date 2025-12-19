clc
clear
close all



list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end

% inertia = 15;

alpha_patch = 0.3;
alpha_plot = 1;

color_tr = [     0 0.4471 0.7412]; %[     0 0.4196 0.6510];
color_bo = [0.8510 0.3255 0.0980]; %[0.8663 0.4061 0.2384];
color_vrft = [0.4667 0.6745 0.1882]; %[0.9294 0.6941 0.1255]; %[0.4846 0.7497 0.5805];
color_model = [0 0 0];

lw = 1.25;

% 
% omegas_tr = [];
% omegas_model = [];
% omegas_vrft = [];
% omegas_bo = [];
% 
% t_tr = [];
% ts_model = [];
% t_vrft = [];
% t_bo = [];
% 
% 
% iqs_tr = [];
% iqs_vrft = [];
% iqs_bo = [];







for inertia = [13, 5, 15, 9, 11, 7]

    
    inertia_name = sprintf("I_%02d",inertia);
    fprintf("inertia %d\n", inertia)
    
    
    switch inertia
        case 13
    
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H13/I13_2025-11-11--11-13-39.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H13/I13_2025-11-11--11-13-39_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_13__ki-0.0000-kp-28.8122/I_13__2025-11-27--10-04-17.csv");
            tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_13__ki-0.0000-kp-2.8007/I_13__2025-11-27--12-17-56.csv");
    
    
        case 5
    
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H05/I05_2025-11-11--11-41-35.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H05/I05_2025-11-11--11-41-35_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_05__ki-0.0152-kp-15.9433/I_05__2025-11-27--14-44-53.csv");
            % tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_");
        
        case 15
    
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H15/I15_2025-11-11--12-10-34.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H15/I15_2025-11-11--12-10-34_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_15__ki-0.0000-kp-11.9834/I_15__2025-11-27--14-48-46.csv");
            tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_15__ki-0.0000-kp-2.9085/I_15__2025-11-27--14-54-14.csv");
    
        case 9
    
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H09/I09_2025-11-11--14-42-05.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H09/I09_2025-11-11--14-42-05_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_09__ki-0.1000-kp-50.0000/I_09__2025-11-27--15-22-39.csv");
            tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_09__ki-0.0000-kp-3.8716/I_09__2025-11-27--15-21-10.csv");
    
        case 11
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H11/I11_2025-11-11--15-10-53.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H11/I11_2025-11-11--15-10-53_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_11__ki-0.0000-kp-43.1595/kp_0.0000_ki_43.1595.csv");
            tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_11__ki-0.0000-kp-3.8905/I_11__2025-11-27--15-36-43.csv");
    
        case 7
    
            tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H07/I07_2025-11-11--15-31-17.csv");
            metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H07/I07_2025-11-11--15-31-17_metadata.mat");
            
            
            tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_07__ki-0.0000-kp-50.0000/I_07__2025-11-27--15-30-24.csv");
            tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_07__ki-0.0000-kp-2.8185/I_07__2025-11-27--15-28-58.csv");
    
    end
    
   
    
    
    % r = tab_tr.r;
    omega = tab_tr.omega;
    
    % start_idx = max(1, find(r>1800,1,"first") - 1);

    if inertia == 13 || inertia == 5
        start_idx = max(1, find(omega>50,1,"first") - 2);
    else

        start_idx = max(1, find(omega>50,1,"first") - 4);
    end
    end_idx = start_idx + 500;
    
    
    omega_cut = tab_tr.omega(start_idx:end_idx);
    iq_cut = tab_tr.iq(start_idx:end_idx);
    t_cut = tab_tr.t(start_idx:end_idx);
    t_cut = t_cut - t_cut(1);
    Ts = t_cut(2)-t_cut(1);
    
    omegas_tr.(inertia_name) = omega_cut;
    iqs_tr.(inertia_name) = iq_cut;
    ts_tr.(inertia_name) = t_cut;
    
    
    OS_percent = metadata.metadata.OS;
    T_set = metadata.metadata.T_s;
    
    s = tf('s');
    z = tf('z',Ts);
    z.Variable = 'z^-1';
    
    OS = OS_percent / 100;
    zeta = -log(OS) / sqrt(pi^2 + (log(OS))^2);
    wn = 3 / (zeta * T_set);
    num = [wn^2];
    den = [1, 2*zeta*wn, wn^2];
    fprintf("Overshoot: %f%%\n", OS_percent)
    fprintf("Settling time: %fs\n", T_set)
    
    
    
    M = tf(num, den);
    M = c2d(M,Ts,'tustin');
    
    
    
    [y_model, t_model] = step(2000*M, 0:Ts:5);
    
    
    omegas_model.(inertia_name) = y_model;
    ts_model.(inertia_name) = t_model;
    
    
    try
        iq_ref = tab_bo.iq_ref;
        start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
        end_idx = start_idx + 5000;
    
    catch
        iq = tab_bo.iq;
        start_idx = max(1, find(iq>0.1,1,"first") - 1);
        end_idx = start_idx + 500;
    end
    
    
    omega_cut_bo = tab_bo.omega(start_idx:end_idx);
    iq_cut_bo = tab_bo.iq(start_idx:end_idx);
    t_cut_bo = tab_bo.t(start_idx:end_idx) - tab_bo.t(start_idx);
    
    omegas_bo.(inertia_name) = omega_cut_bo;
    iqs_bo.(inertia_name) = iq_cut_bo;
    ts_bo.(inertia_name) = t_cut_bo;
    
    
    try
        iq_ref = tab_vrft.iq_ref;
        start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
        end_idx = start_idx + 5000;
        
        omega_cut_vrft = tab_vrft.omega(start_idx:end_idx);
        iq_cut_vrft = tab_vrft.iq(start_idx:end_idx);
        t_cut_vrft = tab_vrft.t(start_idx:end_idx) - tab_vrft.t(start_idx);
    
        omegas_vrft.(inertia_name) = omega_cut_vrft;
        iqs_vrft.(inertia_name) = iq_cut_vrft;
        ts_vrft.(inertia_name) = t_cut_vrft;
    
    
    
    catch
        fprintf("No vrft found\n")
    end



    clear tab_tr metadata tab_bo tab_vrft


end



%%% fix for specific bo I 11

time_tmp = ts_bo.I_13;
omega_new = interp1(ts_bo.I_11, omegas_bo.I_11, time_tmp, "linear","extrap");
iq_new = interp1(ts_bo.I_11, iqs_bo.I_11, time_tmp, "linear","extrap");
ts_bo.I_11 = time_tmp;
omegas_bo.I_11 = omega_new;
iqs_bo.I_11 = iq_new;

% plot(ts_bo.I_11, omegas_bo.I_11)

%%% not sure why i'm doing it this way

omegas_tr_matrix = [];
iqs_tr_matrix = [];

omegas_bo_matrix = [];
iqs_bo_matrix = [];

omegas_vrft_matrix = [];
iqs_vrft_matrix = [];

omegas_model_matrix = [];


for inertia = [5, 13, 15, 9, 11, 7]

    
    inertia_name = sprintf("I_%02d",inertia);

    omegas_tr_matrix(end+1,:) = omegas_tr.(inertia_name);
    iqs_tr_matrix(end+1,:) = iqs_tr.(inertia_name);
    
    omegas_bo_matrix(end+1,:) = omegas_bo.(inertia_name);
    iqs_bo_matrix(end+1,:) = iqs_bo.(inertia_name);
    try
        omegas_vrft_matrix(end+1,:) = omegas_vrft.(inertia_name);
        iqs_vrft_matrix(end+1,:) = iqs_vrft.(inertia_name);
    catch
    end
    omegas_model_matrix(end+1,:) = omegas_model.(inertia_name);

end


time = ts_tr.I_13';

top_layer_tr_omega = max(omegas_tr_matrix, [], 1);
bottom_layer_tr_omega = min(omegas_tr_matrix, [], 1);
top_layer_tr_iq = max(iqs_tr_matrix, [], 1);
bottom_layer_tr_iq = min(iqs_tr_matrix, [], 1);

x_area_tr_omega = [time, fliplr(time)];
in_between_tr_omega = [bottom_layer_tr_omega, fliplr(top_layer_tr_omega)];
x_area_tr_iq = [time, fliplr(time)];
in_between_tr_iq = [bottom_layer_tr_iq, fliplr(top_layer_tr_iq)];

time = ts_bo.I_07';
top_layer_bo_omega = max(omegas_bo_matrix, [], 1);
bottom_layer_bo_omega = min(omegas_bo_matrix, [], 1);
top_layer_bo_iq = max(iqs_bo_matrix, [], 1);
bottom_layer_bo_iq = min(iqs_bo_matrix, [], 1);

x_area_bo_omega = [time, fliplr(time)];
in_between_bo_omega = [bottom_layer_bo_omega, fliplr(top_layer_bo_omega)];
x_area_bo_iq = [time, fliplr(time)];
in_between_bo_iq = [bottom_layer_bo_iq, fliplr(top_layer_bo_iq)];

 

top_layer_vrft_omega = max(omegas_vrft_matrix, [], 1);
bottom_layer_vrft_omega = min(omegas_vrft_matrix, [], 1);
top_layer_vrft_iq = max(iqs_vrft_matrix, [], 1);
bottom_layer_vrft_iq = min(iqs_vrft_matrix, [], 1);

x_area_vrft_omega = [time, fliplr(time)];
in_between_vrft_omega = [bottom_layer_vrft_omega, fliplr(top_layer_vrft_omega)];
x_area_vrft_iq = [time, fliplr(time)];
in_between_vrft_iq = [bottom_layer_vrft_iq, fliplr(top_layer_vrft_iq)];

% figure;
% hold on
% plot(t_cut, omega_cut, 'Color',color_tr, 'LineWidth',1, 'DisplayName', '$\mathcal{C}$')
% plot(t_model, y_model, '--', 'Color', 'k' , 'LineWidth',2, 'DisplayName', '$M$')
% try
% plot(time_vrft, omega_cut_vrft, 'Color',color_vrft, 'LineWidth',1, 'DisplayName', 'VRFT')
% catch
% end
% plot(time_bo, omega_cut_bo, 'Color',color_bo, 'LineWidth',1, 'DisplayName', 'BO')
% tit = sprintf('Step Response ($T_{set}\\approx %.2fs, OS_{\\%%}\\approx %.2f\\%%$)', T_set, OS_percent);
% title(tit, 'Interpreter','latex');
% grid on;
% xlim([0,1.5])
% ylim([0,2500])
% legend('Location','southeast')

fig = figure('WindowStyle', 'normal','Position', [200,200,500,300],'DefaultAxesFontSize',12, 'Renderer', 'painters');
tiles(1) = subplot(2, 1, 1); hold on; box on;
tiles(2) = subplot(2, 1, 2); hold on; box on;
 
% Axes labels

xlabel(tiles(2), 'Time [s]');
ylabel(tiles(1), '$\omega$ [rpm]');
ylabel(tiles(2), '$i_q$ [A]');
 
drawnow;
 
% Set axes dimensions

vert_spacing = 10/fig.Position(4);
tot_width = tiles(1).Position(2) + tiles(1).Position(4) - tiles(2).Position(2);
tiles(2).Position(4) = (tot_width - vert_spacing) *0.4;
tiles(1).Position(2) = tiles(2).Position(2) + tiles(2).Position(4) + vert_spacing;
tiles(1).Position(4) = (tot_width - vert_spacing) *0.6;
tiles(1).XTickLabel = [];
 


% figure('Position',[200,200,500,300],'DefaultAxesFontSize',12)
% 
% subplot(211)
% hold on
% box on

% patch(x_area_vrft_omega, in_between_vrft_omega, color_vrft, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_vrft = color_tints_and_shades(color_vrft,size(omegas_vrft_matrix, 1)+2, 0.4);
for i = 1:size(omegas_vrft_matrix, 1)
plot(tiles(1), ts_vrft.I_13, omegas_vrft_matrix(i,:), 'Color',[colors_vrft{i}], 'LineWidth',lw)
end

% patch(x_area_bo_omega, in_between_bo_omega, color_bo, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_bo = color_tints_and_shades(color_bo,size(omegas_bo_matrix, 1)+3, 0.4);
tmp = colors_bo{1};
colors_bo{1} = colors_bo{2};
colors_bo{2} = tmp;

for i = 1:size(omegas_bo_matrix, 1)
plot(tiles(1), ts_bo.I_13, omegas_bo_matrix(i,:), 'Color',colors_bo{i}, 'LineWidth',lw + 0.5)
end

% patch(x_area_tr_omega, in_between_tr_omega, color_tr, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_tr = color_tints_and_shades([0    0.4471    0.7412],8,0.65);
% cell2mat()
% tmp = colors_tr{1};
% colors_tr{1} = colors_tr{2};
% colors_tr{2} = tmp;
for i = 1:size(omegas_tr_matrix, 1)
plot(tiles(1), ts_tr.I_13, omegas_tr_matrix(i,:), 'Color',[colors_tr{i}], 'LineWidth',lw+0.75,'LineStyle','-.')
end

% plot(tiles(1), ts_model.I_13, omegas_model_matrix, 'Color',[color_model], 'LineStyle', '--', 'LineWidth',lw)
% tit = sprintf('Step Response ($T_{set}\\approx %.2fs, OS_{\\%%}\\approx %.2f\\%%$)', T_set, OS_percent);
% title(tit, 'Interpreter','latex');
% ylabel("$\omega$  [rpm]")
xlim(tiles(1), [0,1.5])
ylim(tiles(1), [0,2500])


L_tr = plot(tiles(1), nan, nan, 'color', [color_tr, alpha_plot],'LineStyle','-.', 'DisplayName', "$\mathcal{C}$", 'LineWidth',lw);
% L_model = plot(tiles(1), nan, nan, 'color', color_model, 'LineStyle', '--', 'DisplayName', "$M$", 'LineWidth',lw);
L_bo = plot(tiles(1), nan, nan, 'color', [color_bo, alpha_plot], 'DisplayName', "BO", 'LineWidth',lw);
L_vrft = plot(tiles(1), nan, nan, 'color', [color_vrft, alpha_plot], 'DisplayName', "VRFT", 'LineWidth',lw);
% legend(tiles(1), [L_tr, L_model, L_bo, L_vrft],'Location','southeast')
legend(tiles(1), [L_tr, L_bo, L_vrft],'Location','southeast')

% subplot(212)
% hold on
% box on



% patch(x_area_vrft_omega, in_between_vrft_omega, color_vrft, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_vrft = color_tints_and_shades(color_vrft,size(iqs_vrft_matrix, 1)+2, 0.4);
for i = 1:size(iqs_vrft_matrix, 1)
plot(tiles(2), ts_vrft.I_13, iqs_vrft_matrix(i,:), 'Color',[colors_vrft{i}], 'LineWidth',lw)
end

% patch(x_area_bo_omega, in_between_bo_omega, color_bo, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_bo = color_tints_and_shades(color_bo,size(iqs_bo_matrix, 1)+3, 0.4);
% tmp = colors_bo{1};
% colors_bo{1} = colors_bo{2};
% colors_bo{2} = tmp;

for i = 1:size(iqs_bo_matrix, 1)
plot(tiles(2), ts_bo.I_13, iqs_bo_matrix(i,:), 'Color',colors_bo{i}, 'LineWidth',lw + 0.5)
end

% patch(x_area_tr_omega, in_between_tr_omega, color_tr, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
colors_tr = color_tints_and_shades(color_tr,size(iqs_tr_matrix, 1)+2, 0.4);
tmp = colors_tr{1};
colors_tr{1} = colors_tr{2};
colors_tr{2} = tmp;
for i = 1:size(iqs_tr_matrix, 1)
plot(tiles(2), ts_tr.I_13, iqs_tr_matrix(i,:), 'Color',[colors_tr{i}], 'LineWidth',lw+0.75,'LineStyle','-.')
end
% 
% % patch(x_area_bo_iq, in_between_bo_iq, color_bo, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
% plot(tiles(2), ts_bo.I_13, iqs_bo_matrix, 'Color',[color_bo, alpha_plot], 'LineWidth',lw)
% 
% % patch(x_area_vrft_iq, in_between_vrft_iq, color_vrft, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
% plot(tiles(2), ts_vrft.I_13, iqs_vrft_matrix, 'Color',[color_vrft, alpha_plot], 'LineWidth',lw)
% 
% % patch(x_area_tr_iq, in_between_tr_iq, color_tr, 'FaceAlpha',alpha_patch, 'EdgeColor', 'none')
% plot(tiles(2), ts_tr.I_13, iqs_tr_matrix, 'Color',[color_tr, alpha_plot], 'LineWidth',lw)

% tit = sprintf('Step Response ($T_{set}\\approx %.2fs, OS_{\\%%}\\approx %.2f\\%%$)', T_set, OS_percent);
% title(tit, 'Interpreter','latex');
% ylabel("$i_q$  [A]")
% xlabel("Time [s]")
xlim(tiles(2), [0,1.5])
% ylim([0,2500])


% L_tr = plot(tiles(2), nan, nan, 'color', [color_tr, alpha_plot], 'DisplayName', "$\mathcal{C}$", 'LineWidth',lw);
% L_bo = plot(tiles(2), nan, nan, 'color', [color_bo, alpha_plot], 'DisplayName', "BO", 'LineWidth',lw);
% L_vrft = plot(tiles(2), nan, nan, 'color', [color_vrft, alpha_plot], 'DisplayName', "VRFT", 'LineWidth',lw);
% legend(tiles(2), [L_tr, L_bo, L_vrft],'Location','northeast')



savefig("figs_paper/comp.fig")
saveas(gcf, "figs_paper/comp.png")
fig = gcf;
set(fig, 'PaperPositionMode', 'auto');
exportgraphics(fig, 'figs_paper/comp.pdf', 'ContentType', 'vector');





% 
% fig = figure('WindowStyle', 'normal','Position', [200,200,1000,200],'DefaultAxesFontSize',12, 'Renderer', 'painters');
% 
% for i = 1:6
% 
% tiles(1) = subplot(2, 6, i); hold on; box on;
% tiles(2) = subplot(2, 6, 6+i); hold on; box on;
% 
% 
% vert_spacing = 10/fig.Position(4);
% tot_width = tiles(1).Position(2) + tiles(1).Position(4) - tiles(2).Position(2);
% tiles(2).Position(4) = (tot_width - vert_spacing) *0.4;
% tiles(1).Position(2) = tiles(2).Position(2) + tiles(2).Position(4) + vert_spacing;
% tiles(1).Position(4) = (tot_width - vert_spacing) *0.6;
% 
% 
% 
% tiles(1).XTickLabel = [];
% if i>1
%     tiles(1).YTickLabel = [];
%     tiles(2).YTickLabel = [];
% else
%     ylabel(tiles(1), '$\omega$ [rpm]');
%     ylabel(tiles(2), '$i_q$ [A]');
% end
% 
% xlabel(tiles(2), 'Time [s]');
% 
% xlim(tiles(1), [0,1.5])
% xlim(tiles(2), [0,1.5])
% ylim(tiles(1), [0,2500])
% % ylim(tiles(1), [0,2500])
% ylim(tiles(2), [-2.5,5])
% 
% try
% plot(tiles(1), ts_vrft.I_13, omegas_vrft_matrix(i-1,:), 'Color',[colors_vrft{i}], 'LineWidth',lw, 'DisplayName', "VRFT")
% catch
% end
% plot(tiles(1), ts_bo.I_13, omegas_bo_matrix(i,:), 'Color',colors_bo{i}, 'LineWidth',lw + 0.5, 'DisplayName', "BO")
% plot(tiles(1), ts_tr.I_13, omegas_tr_matrix(i,:), 'Color',[colors_tr{i}], 'LineWidth',lw+0.75,'LineStyle','-.', 'DisplayName', "$\mathcal{C}$")
% 
% 
% 
% try
% plot(tiles(2), ts_vrft.I_13, iqs_vrft_matrix(i-1,:), 'Color',[colors_vrft{i}], 'LineWidth',lw, 'DisplayName', "VRFT")
% catch
% end
% plot(tiles(2), ts_bo.I_13, iqs_bo_matrix(i,:), 'Color',colors_bo{i}, 'LineWidth',lw + 0.5, 'DisplayName', "BO")
% plot(tiles(2), ts_tr.I_13, iqs_tr_matrix(i,:), 'Color',[colors_tr{i}], 'LineWidth',lw+0.75,'LineStyle','-.', 'DisplayName', "$\mathcal{C}$")
% leg = legend(tiles(1),'Location','southeast');
% leg.IconColumnWidth = 15;
% end
% 


%%


fig = figure('WindowStyle', 'normal','Position', [200,200,1000,200],'DefaultAxesFontSize',12, 'Renderer', 'painters');


% --- Configuration for Horizontal Spacing ---
n_cols = 6;
left_margin = 0.06;   % Space on the far left (for Y-axis labels)
right_margin = 0.01;  % Space on the far right
gap_width = 0.01;     % Horizontal gap between columns
% Calculate the width of a single axes
plot_width = (1 - left_margin - right_margin - (n_cols-1)*gap_width) / n_cols;
% --------------------------------------------

clearvars tiles
for i = 1:6
    tiles(i) = subplot(2, 6, i); hold on; box on; grid off;
    tiles(i+6) = subplot(2, 6, 6+i); hold on; box on; grid off;
    tiles(i).XTickLabel = [];
    if i>1
        tiles(i).YTickLabel = [];
        tiles(i+6).YTickLabel = [];
    else
        ylabel(tiles(i), '$\omega$ [rpm]');
        ylabel(tiles(i+6), '$i_q$ [A]');
    end
end
xl = xlabel(tiles(7), 'Time [s]');
drawnow;
xl.Visible = 'off';
tiles(1).Position(1) = tiles(1).Position(1)/2;
tiles(6).Position(1) = 1 - tiles(1).Position(1) - tiles(6).Position(3);
tiles(1).Position(4) = tiles(1).Position(4)*0.8;

for i = 1:6

    vert_spacing = 10/fig.Position(4);
    tot_height = tiles(1).Position(2) + tiles(1).Position(4) - tiles(7).Position(2);
    tiles(i+6).Position(4) = (tot_height - vert_spacing) *0.4;
    tiles(i).Position(2) = tiles(i+6).Position(2) + tiles(i+6).Position(4) + vert_spacing;
    tiles(i).Position(4) = (tot_height - vert_spacing) *0.6;
    
    hor_spacing = 10/fig.Position(3);
    width = (tiles(6).Position(1) + tiles(6).Position(3) - tiles(1).Position(1) - 5*hor_spacing)/6;
    tiles(i).Position(3) = width;
    tiles(i+6).Position(3) = width;
    tiles(i).Position(1) = (i-1)*(width+hor_spacing) + tiles(1).Position(1);
    tiles(i+6).Position(1) = (i-1)*(width+hor_spacing) + tiles(1).Position(1);

end
drawnow;
t_tmp = axes;
t_tmp.Position(1) = tiles(7).Position(1);
t_tmp.Position(3) = tiles(end).Position(1)+tiles(end).Position(3)-tiles(7).Position(1);
t_tmp.Position(2) = tiles(7).Position(2);
xlabel(t_tmp, 'Time [s]');
t_tmp.Visible = 'off';
t_tmp.XLabel.Visible = 'on';
t_tmp.XLabel.Position(2) = t_tmp.XLabel.Position(2)*0.66;


% x_pos = left_margin + (i-1)*(plot_width + gap_width);
% 
% tiles(1).Position(1) = x_pos;
% tiles(1).Position(3) = plot_width;
% tiles(2).Position(1) = x_pos;
% tiles(2).Position(3) = plot_width;


curr_color_vrft = lighten_color(color_vrft, 0.25);
curr_color_bo = lighten_color(color_bo, 0.2);
curr_color_tr = lighten_color(color_tr, 0.2);

for i = 1:6


xlim(tiles(i), [0,1])
xlim(tiles(i+6), [0,1])
ylim(tiles(i), [0,2500])
% ylim(tiles(i), [0,2500])
ylim(tiles(i+6), [-2.5,5])

tiles(i+6).XTickLabel{1} = ['\,\,', tiles(i+6).XTickLabel{1}];
tiles(i+6).XTickLabel{end} = [tiles(i+6).XTickLabel{end}, ''];
plot(tiles(i), ts_model.I_13, omegas_model_matrix(i,:), 'Color',color_model, 'LineStyle', '--', 'LineWidth',lw, 'DisplayName', "$M_r$")

plot(tiles(i), ts_bo.I_13, omegas_bo_matrix(i,:), 'Color',curr_color_bo, 'LineWidth',lw, 'DisplayName', "BO")
plot(tiles(i), ts_tr.I_13, omegas_tr_matrix(i,:), 'Color',curr_color_tr, 'LineWidth',lw+1.0,'LineStyle',':', 'DisplayName', "ICC (ours)");
try
plot(tiles(i), ts_vrft.I_13, omegas_vrft_matrix(i-1,:), 'Color',curr_color_vrft, 'LineWidth',lw, 'DisplayName', "VRFT")
catch
end

% fprintf("config %d\n", i)
% fprintf("tr rmse = %g\n", rmse(omegas_tr_matrix(i,:), ones(size(omegas_tr_matrix(i,:)))*2000 ))
% fprintf("bo rmse = %g\n", rmse(omegas_bo_matrix(i,:), ones(size(omegas_bo_matrix(i,:)))*2000 ))
% try
%     fprintf("vrft rmse = %g\n", rmse(omegas_vrft_matrix(i-1,:), ones(size(omegas_vrft_matrix(i-1,:)))*2000 ))
% catch
%     fprintf("vrft not avaliable\n")
% end
% fprintf("\n")


model_1000 = interp1(ts_model.I_05, omegas_model_matrix(i,:), ts_vrft.I_13, "linear","extrap");
tr_1000 = interp1(ts_tr.I_05, omegas_tr_matrix(i,:), ts_vrft.I_13, "linear","extrap");

fprintf("config %d\n", i)
% fprintf("tr rmse = %g\n", rmse(omegas_tr_matrix(i,1:end-1), omegas_model_matrix(i,:) ))
fprintf("tr rmse = %g\n", rmse(tr_1000(1:end-10), model_1000(1:end-10) ))
fprintf("bo rmse = %g\n", rmse(omegas_bo_matrix(i,1:end-10), model_1000(1:end-10)' ))
try
    fprintf("vrft rmse = %g\n", rmse(omegas_vrft_matrix(i-1,1:end-10), model_1000(1:end-10)' ))
catch
    fprintf("vrft not avaliable\n")
end
fprintf("\n")





str = sprintf("$S^{(%d)}$", i);
text(tiles(i), 0.94, 0.12, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'none', ... 
    'EdgeColor', 'k', ...
    'FontSize', 10, ...
    'Color','w');
text(tiles(i), 0.94, 0.1, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'none', ... 
    'EdgeColor', 'none', ...
    'Color', 'k', ...
    'FontSize', 10);



plot(tiles(i+6), ts_bo.I_13, iqs_bo_matrix(i,:), 'Color',curr_color_bo, 'LineWidth',lw, 'DisplayName', "BO")
plot(tiles(i+6), ts_tr.I_13, iqs_tr_matrix(i,:), 'Color',curr_color_tr, 'LineWidth',lw+1.0,'LineStyle',':', 'DisplayName', "ICC (ours)")
try
plot(tiles(i+6), ts_vrft.I_13, iqs_vrft_matrix(i-1,:), 'Color',curr_color_vrft, 'LineWidth',lw, 'DisplayName', "VRFT")
catch
end

end

leg = legend(tiles(3),'Location','southeast', 'Orientation','horizontal');
leg.IconColumnWidth = 15;

tmp_size = (tiles(6).Position(1) + tiles(6).Position(3) - tiles(1).Position(1) - leg.Position(3))/2;
leg.Position(1) = tiles(1).Position(1) + tmp_size;
leg.Position(2) = tiles(1).Position(2) + tiles(1).Position(4) + vert_spacing/2;

t_tmp.XLabel.FontSize = xl.FontSize;



savefig("figs_paper/comp_wide.fig")
saveas(gcf, "figs_paper/comp_wide.png")
fig = gcf;
set(fig, 'PaperPositionMode', 'auto');
exportgraphics(fig, 'figs_paper/comp_wide.pdf', 'ContentType', 'vector');