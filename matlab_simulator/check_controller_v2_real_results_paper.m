clc
% close all
clear


list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end


% models_to_test = ["new_dataset_long_noise_h10_40k_H10H", ...
%                   "new_dataset_long_noise_scaled_h10_30k_H10H", ...
%                   "new_dataset_short_noise_h10_30k_H10H", ...
%                   "new_dataset_short_noise_scaled_h10_40k_H10H"];
models_to_test = ["new_dataset_short_noise_h10_30k_H10H"];

n_models = length(models_to_test);

final_scores = zeros(n_models, 6, 2, 5);




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
                    idx = 2;

                case '05'
                    fprintf("config 2\n")
                    idx = 1;

                case '15'
                    fprintf("config 3\n")
                    idx = 3;

                case '09'
                    fprintf("config 4\n")
                    idx = 4;

                case '11'
                    fprintf("config 5\n")
                    idx = 5;

                case '07'
                    fprintf("config 6\n")
                    idx = 6;

                otherwise
                    fprintf("............\n............\nerror\n............\n............\n")
                    error("aaaaaaaaaaaa")

            end

            current_path = fullfile(folder_list(j).folder, folder_list(j).name);
            current_file_list = dir(sprintf('%s',current_path));
            
            del_log_list = [];
            
            for ii = 1:length(current_file_list)
                if strcmp(current_file_list(ii).name,'desktop.ini') | strcmp(current_file_list(ii).name,'.') | strcmp(current_file_list(ii).name,'..') | strcmp(current_file_list(ii).name,'.DS_Store' ) | ~contains(current_file_list(ii).name, 'metadata')      % 'desktop.ini' is a hidden file created by google drive in each directory. 
                    del_log_list(end+1) = ii;                                                                                    % It is deleted from the list since it is not useful for the program.
                end
            end
            
            current_file_list(del_log_list) = [];

            for jj = 1:length(current_file_list)

                current_file = fullfile(current_file_list(jj).folder, current_file_list(jj).name);
                res_tmp = load(current_file);

                final_scores(i, idx, 1, jj) = res_tmp.metadata.T_s;
                final_scores(i, idx, 2, jj) = res_tmp.metadata.OS;

            end
            





            




        else
            fprintf("no match\n")
        end
    
    
    
    
    
    
    
    end %% folder_list



end %% n_models



%%%% final_scores = model, config, [Ts, OS], exp);
% for j = 1:6
% 
% 
%     figure
%     hold on
%     for i = 1:n_models
%         scatter(reshape(final_scores(i,j,1,1:end), [], length(final_scores(i,j,1,1:end))), ...
%                 reshape(final_scores(i,j,2,1:end), [], length(final_scores(i,j,2,1:end))), ...
%                 'filled', 'DisplayName',models_to_test(i))
%     end
%     xline(1.5, 'HandleVisibility','off')
%     yline(20, 'HandleVisibility','off')
%     yl = ylim;
%     ylim([min(0, yl(1)), max(yl(2),100)])
%     xlim([0,10])
%     xlabel("Settling time [s]")
%     ylabel("Overshoot percentage [%]")
%     legend('Interpreter','none', Location='northoutside')
%     title(sprintf("Config %d", j))
%     savefig(sprintf("exp_results_CAN/result_config%d.fig", j))
%     saveas(gcf, sprintf("exp_results_CAN/result_config%d.png", j))
% 
% 
% end


% colors = ["#FAA307", "#FFBA08", "#F48C06", "#E85D04", "#DC2F02", "#D00000"];
colors = ["#023E8A", "#03045E", "#0077B6", "#0096C7", "#00B4D8", "#48CAE4"];

% colors = cell2mat(color_tints_and_shades(cell2mat(hex2rgb("#0077B6")),10,0.5));
colors = cell2mat(color_tints_and_shades([0    0.4471    0.7412],8,0.65));

% tmp = colors(1,:);
% colors(1,:) = colors(2,:);
% colors(2,:) = tmp;

% colors = colors(end-5:end,:);


% 
% 
% for i = 1:n_models
% 
%     figure
%     hold on
% 
%     for j = 1:6
%         scatter(reshape(final_scores(i,j,1,1:end), [], length(final_scores(i,j,1,1:end))), ...
%                 reshape(final_scores(i,j,2,1:end), [], length(final_scores(i,j,2,1:end))), ...
%                 'filled', 'DisplayName', sprintf("Config %d", j),  'MarkerFaceColor',colors(j))
%     end
%     xline(1.5, 'HandleVisibility','off')
%     yline(20, 'HandleVisibility','off')
%     yl = ylim;
%     ylim([min(0, yl(1)), max(yl(2),100)])
%     xlim([0,10])
%     r = rectangle('Position',[0,0,1.5,20]);
%     r.EdgeColor = '#007200';
%     r.LineStyle = ":";
%     r.LineWidth = 3;
%     xlabel("Settling time [s]")
%     ylabel("Overshoot percentage [\%]")
%     legend('Location', 'north','NumColumns',6)
%     % title(models_to_test(i), Interpreter="none")
%     savefig("figs_paper/scatter_results.fig")
%     saveas(gcf, "figs_paper/scatter_results.png")
%     fig = gcf;
%     set(fig, 'PaperPositionMode', 'auto');
%     exportgraphics(fig, 'figs_paper/scatter_results.pdf', 'ContentType', 'vector');
% 
% end

load("results_simulator.mat")
% color_sim = "#38b000";
% color_sim = [0.4846 0.7497 0.5805];

color_sim = [0.8663 0.4061 0.2384];
color_sim = lighten_color(color_sim, 0.2);


for i = 1:n_models

    figure('Position',[100,100,500,200])
    box on
    hold on

    scatter(metadata_full(:,2), metadata_full(:,1), 35, ...
        'DisplayName', "Sim", 'MarkerFaceAlpha',1, 'MarkerEdgeColor',color_sim, ...
        'Marker','.', 'MarkerFaceColor',lighten_color(color_sim, 0.6), 'LineWidth', 1)
    for j = 1:6
        scatter(reshape(final_scores(i,j,1,1:end), [], length(final_scores(i,j,1,1:end))), ...
                reshape(final_scores(i,j,2,1:end), [], length(final_scores(i,j,2,1:end))), ...
                25,'filled', 'DisplayName', sprintf("$S^{(%d)}$", j), 'MarkerFaceColor',colors(j,:), 'MarkerFaceAlpha',1)
    end
    xline(1.5, 'HandleVisibility','off')
    yline(20, 'HandleVisibility','off')
    % yl = ylim;
    % ylim([min(0, yl(1)), max(yl(2),100)])
    ylim([0 30])
    xlim([0,2])
    r = rectangle('Position',[0,0,1.5,20]);
    r.EdgeColor = '#007200';
    r.LineStyle = ":";
    r.LineWidth = 3;
    xlabel("$T_\mathrm{set}$ [s]")
    ylabel("$OS_\%$ [$\%$]")
    legend('Location', 'north','NumColumns',7)
    % title(models_to_test(i), Interpreter="none")
    savefig("figs_paper/scatter_results_and_sim.fig")
    saveas(gcf, "figs_paper/scatter_results_and_sim.png")
    fig = gcf;
    set(fig, 'PaperPositionMode', 'auto');
    exportgraphics(fig, 'figs_paper/scatter_results_and_sim.pdf', 'ContentType', 'vector');




end



