clear
clc
close all


% models_to_test = ["new_dataset_long_noise_h10_40k_H10H", ...
%                   "new_dataset_long_noise_scaled_h10_30k_H10H", ...
%                   "new_dataset_short_noise_h10_30k_H10H", ...
%                   "new_dataset_short_noise_scaled_h10_40k_H10H"];
models_to_test = ["new_dataset_short_noise_h10_30k_H10H"];

n_models = length(models_to_test);


time_stack = [];
omega_stack = [];
color_stack = strings(0);
color_stack_good = strings(0);
stepsize = 2000;





temp_name = strsplit(pwd,'in-context-bldc');
user_tmp = strsplit(pwd,'Users\');
user_tmp2 = strsplit(user_tmp{2},'\');
user = user_tmp2{1};



datapath = fullfile(temp_name{1}, 'in-context-bldc', 'data', 'transformer_v2_CAN_exp');


folder_list = dir(sprintf('%s',datapath));

del_log_list = [];

for i = 1:length(folder_list)
    if strcmp(folder_list(i).name,'desktop.ini') | strcmp(folder_list(i).name,'.') | strcmp(folder_list(i).name,'..') | strcmp(folder_list(i).name,'.DS_Store' )      % 'desktop.ini' is a hidden file created by google drive in each directory. 
        del_log_list(end+1) = i;                                                                                    % It is deleted from the list since it is not useful for the program.
    end
end

folder_list(del_log_list) = [];


n_folders = length(folder_list);


RGB = orderedcolors("gem");
H = rgb2hex(RGB);


for i = 1:n_models


    for j = 1:n_folders
        name_tmp = strsplit(folder_list(j).name, "\\") ;
        last_folder = string(name_tmp{end});
        fprintf("looking for %s in %s: ", models_to_test(i), last_folder)

        if contains(last_folder, models_to_test(i))
            fprintf("match\n")
            inertia_str = char(name_tmp);
            inertia_str = inertia_str(end-1:end);
            fprintf("%s\n", inertia_str)

            switch inertia_str
                case '13'
                    fprintf("config 1\n")
                    idx = 1;
                    color_good = "#FAA307";
                    color = H(1);

                case '05'
                    fprintf("config 2\n")
                    idx = 2;
                    color_good = "#FFBA08";
                    color = H(2);

                case '15'
                    fprintf("config 3\n")
                    idx = 3;
                    color_good = "#F48C06";
                    color = H(3);

                case '09'
                    fprintf("config 4\n")
                    idx = 4;
                    color_good = "#E85D04";
                    color = H(4);

                case '11'
                    fprintf("config 5\n")
                    idx = 5;
                    color_good = "#DC2F02";
                    color = H(5);

                case '07'
                    fprintf("config 6\n")
                    idx = 6;
                    color_good = "#D00000";
                    color = H(6);

                otherwise
                    fprintf("............\n............\nerror\n............\n............\n")
                    error("aaaaaaaaaaaa")

            end

            current_path = fullfile(folder_list(j).folder, folder_list(j).name);
            current_file_list = dir(sprintf('%s',current_path));
            
            del_log_list = [];
            
            for ii = 1:length(current_file_list)
                if strcmp(current_file_list(ii).name,'desktop.ini') | strcmp(current_file_list(ii).name,'.') | strcmp(current_file_list(ii).name,'..') | strcmp(current_file_list(ii).name,'.DS_Store' ) | ~contains(current_file_list(ii).name, '.csv')      % 'desktop.ini' is a hidden file created by google drive in each directory. 
                    del_log_list(end+1) = ii;                                                                                    % It is deleted from the list since it is not useful for the program.
                end
            end
            
            current_file_list(del_log_list) = [];

            for jj = 1:length(current_file_list)

                current_file = fullfile(current_file_list(jj).folder, current_file_list(jj).name);
                res_tmp = readtable(current_file);
                r = res_tmp.r;
                start_idx = max(1, find(r>1800,1,"first") - 1);
                % end_idx = min(length(r), find(r>1800,1,"last"));
                end_idx = start_idx + 500;

                time_stack(end+1,:) = res_tmp.t(start_idx:end_idx) - res_tmp.t(start_idx);
                omega_stack(end+1,:) = res_tmp.omega(start_idx:end_idx);
                color_stack(end+1) = color;
                color_stack_good(end+1) = color_good;

            end
            





            




        else
            fprintf("no match\n")
        end
    
    
    
    
    
    
    
    end %% folder_list



end %% n_models


[n_exp, ~] = size(omega_stack);


figure
hold on
for j = 1:n_exp
    plot(time_stack(j,:), omega_stack(j,:), 'Color',color_stack(j))
end

% r = rectangle('Position',[1.5, 0, 5, 1900], 'FaceColor',"b","EdgeColor","none","FaceAlpha",0.1);
% hatchfill(r, 'single',-45, 'b');
x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [0, 0, 1900, 1900];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)

x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [2100, 2100, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)


x_corners = [0, 1.5, 1.5, 0]; 
y_corners = [2400, 2400, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)

xlim([0,5])
ylim([0,2500])
xlabel("Time [s]")
ylabel("Speed [rpm]")


% 
% L1 = plot(nan, nan, 'color', "#FAA307", 'LineWidth',2);
% L2 = plot(nan, nan, 'color', "#FFBA08", 'LineWidth',2);
% L3 = plot(nan, nan, 'color', "#F48C06", 'LineWidth',2);
% L4 = plot(nan, nan, 'color', "#E85D04", 'LineWidth',2);
% L5 = plot(nan, nan, 'color', "#DC2F02", 'LineWidth',2);
% L6 = plot(nan, nan, 'color', "#D00000", 'LineWidth',2);
L1 = plot(nan, nan, 'color', H(1), 'LineWidth',2);
L2 = plot(nan, nan, 'color', H(2), 'LineWidth',2);
L3 = plot(nan, nan, 'color', H(3), 'LineWidth',2);
L4 = plot(nan, nan, 'color', H(4), 'LineWidth',2);
L5 = plot(nan, nan, 'color', H(5), 'LineWidth',2);
L6 = plot(nan, nan, 'color', H(6), 'LineWidth',2);
legend([L1, L2,L3,L4,L5,L6], {'Config 1', 'Config 2', 'Config 3', 'Config 4' ,'Config 5' ,'Config 6'}, 'Location','southeast')

savefig("figs_paper/stack_results_bad_color.fig")
saveas(gcf, "figs_paper/stack_results_bad_color.png")
fig = gcf;
set(fig, 'PaperPositionMode', 'auto');
exportgraphics(fig, 'figs_paper/stack_results_bad_color.pdf', 'ContentType', 'vector');








figure('Position',[100,100,500,300])
hold on
for j = 1:n_exp
    plot(time_stack(j,:), omega_stack(j,:), 'Color',color_stack_good(j))
end

% r = rectangle('Position',[1.5, 0, 5, 1900], 'FaceColor',"b","EdgeColor","none","FaceAlpha",0.1);
% hatchfill(r, 'single',-45, 'b');
x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [0, 0, 1900, 1900];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)

x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [2100, 2100, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)


x_corners = [0, 1.5, 1.5, 0]; 
y_corners = [2400, 2400, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', 'b', 'LineWidth', 0.5)

xlim([0,5])
ylim([0,2500])
xlabel("Time [s]")
ylabel("Speed [rpm]")


% 
L1 = plot(nan, nan, 'color', "#FAA307", 'LineWidth',2);
L2 = plot(nan, nan, 'color', "#FFBA08", 'LineWidth',2);
L3 = plot(nan, nan, 'color', "#F48C06", 'LineWidth',2);
L4 = plot(nan, nan, 'color', "#E85D04", 'LineWidth',2);
L5 = plot(nan, nan, 'color', "#DC2F02", 'LineWidth',2);
L6 = plot(nan, nan, 'color', "#D00000", 'LineWidth',2);
% L1 = plot(nan, nan, 'color', H(1), 'LineWidth',2);
% L2 = plot(nan, nan, 'color', H(2), 'LineWidth',2);
% L3 = plot(nan, nan, 'color', H(3), 'LineWidth',2);
% L4 = plot(nan, nan, 'color', H(4), 'LineWidth',2);
% L5 = plot(nan, nan, 'color', H(5), 'LineWidth',2);
% L6 = plot(nan, nan, 'color', H(6), 'LineWidth',2);
legend([L1, L2,L3,L4,L5,L6], {'$S^1$', '$S^2$', '$S^3$', '$S^4$' ,'$S^5$' ,'$S^6$'}, 'Location','southeast', 'Interpreter','latex')

savefig("figs_paper/stack_results.fig")
saveas(gcf, "figs_paper/stack_results.png")
fig = gcf;
set(fig, 'PaperPositionMode', 'auto');
exportgraphics(fig, 'figs_paper/stack_results.pdf', 'ContentType', 'vector');




% 
% figure
% hold on
% 
% load("results_simulator_stack.mat")
% 
% [n_exp_sim, t_len] = size(out.omegas);
% 
% mask = 1:10:n_exp_sim;
% 
% n_colors = n_exp_sim;
% 
% 
% start_color_g = [0, 0.5, 0];
% end_color_g   = [0.5, 0.8, 0.5];
% 
% % Create the gradient for each channel
% r_channel = linspace(start_color_g(1), end_color_g(1), n_colors)';
% g_channel = linspace(start_color_g(2), end_color_g(2), n_colors)';
% b_channel = linspace(start_color_g(3), end_color_g(3), n_colors)';
% colors = [r_channel, g_channel, b_channel];
% 
% for j = 1:10:n_exp_sim
%     plot(out.time, out.omegas(j,:), 'Color',colors(j,:),'LineWidth',0.1);
%     % alpha(p, 0.5)
% end
% 
% 
% 
% for j = 1:n_exp
%     plot(time_stack(j,:), omega_stack(j,:), 'Color',color_stack_good(j))
% end
% 
% % r = rectangle('Position',[1.5, 0, 5, 1900], 'FaceColor',"b","EdgeColor","none","FaceAlpha",0.1);
% % hatchfill(r, 'single',-45, 'b');
% x_corners = [1.5, 5, 5, 1.5]; 
% y_corners = [0, 0, 1900, 1900];
% p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
% hh = hatchfill(p, 'single', 45, 5);
% set(hh, 'Color', 'b', 'LineWidth', 0.5)
% 
% x_corners = [1.5, 5, 5, 1.5]; 
% y_corners = [2100, 2100, 3000, 3000];
% p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
% hh = hatchfill(p, 'single', 45, 5);
% set(hh, 'Color', 'b', 'LineWidth', 0.5)
% 
% 
% x_corners = [0, 1.5, 1.5, 0]; 
% y_corners = [2400, 2400, 3000, 3000];
% p = patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% patch(x_corners, y_corners, 'r', 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
% hh = hatchfill(p, 'single', 45, 5);
% set(hh, 'Color', 'b', 'LineWidth', 0.5)
% 
% xlim([0,5])
% ylim([0,2500])
% xlabel("Time [s]")
% ylabel("Speed [rpm]")
% 
% 
% % 
% L1 = plot(nan, nan, 'color', "#FAA307", 'LineWidth',2);
% L2 = plot(nan, nan, 'color', "#FFBA08", 'LineWidth',2);
% L3 = plot(nan, nan, 'color', "#F48C06", 'LineWidth',2);
% L4 = plot(nan, nan, 'color', "#E85D04", 'LineWidth',2);
% L5 = plot(nan, nan, 'color', "#DC2F02", 'LineWidth',2);
% L6 = plot(nan, nan, 'color', "#D00000", 'LineWidth',2);
% % L1 = plot(nan, nan, 'color', H(1), 'LineWidth',2);
% % L2 = plot(nan, nan, 'color', H(2), 'LineWidth',2);
% % L3 = plot(nan, nan, 'color', H(3), 'LineWidth',2);
% % L4 = plot(nan, nan, 'color', H(4), 'LineWidth',2);
% % L5 = plot(nan, nan, 'color', H(5), 'LineWidth',2);
% % L6 = plot(nan, nan, 'color', H(6), 'LineWidth',2);
% legend([L1, L2,L3,L4,L5,L6], {'Config 1', 'Config 2', 'Config 3', 'Config 4' ,'Config 5' ,'Config 6'}, 'Location','southeast')
% 
% savefig("figs_paper/stack_results_and_sim.fig")
% saveas(gcf, "figs_paper/stack_results_and_sim.png")
% fig = gcf;
% set(fig, 'PaperPositionMode', 'auto');
% exportgraphics(fig, 'figs_paper/stack_results_and_sim.pdf', 'ContentType', 'vector');