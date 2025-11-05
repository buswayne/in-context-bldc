clear
clc
close all


temp_name = strsplit(pwd,'in-context-bldc');
user_tmp = strsplit(pwd,'Users\');
user_tmp2 = strsplit(user_tmp{2},'\');
user = user_tmp2{1};


model_name = 'new_dataset_short_noise_h10_30k_H10H.mat';
savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
folder_name = sprintf('statistical_analysis_model_%s', model_name(1:end-4));

datapath = fullfile(savepath_tmp, folder_name);
addpath(datapath)

file_list = dir(sprintf('%s',datapath));

del_log_list = [];

for i = 1:length(file_list)
    if strcmp(file_list(i).name,'desktop.ini') | strcmp(file_list(i).name,'.') | strcmp(file_list(i).name,'..') | strcmp(file_list(i).name,'.DS_Store' ) | ~contains(file_list(i).name, 'results')      % 'desktop.ini' is a hidden file created by google drive in each directory. 
        del_log_list(end+1) = i;                                                                                    % It is deleted from the list since it is not useful for the program.
    end
end

file_list(del_log_list) = [];

stepsize = 2000;
eps = 0.05 * stepsize;
T_s_th = 1.5;


total_exp = length(file_list);
success_counter = 0;
limited_speed_counter = 0;
slow_sys_counters = 0;
fast_sys_counter = 0;
just_bad_counter = 0;

show_bad = false;
show_good = true;
show_bad_counter = 0; 

OS_list = zeros(total_exp,2);

plot_example = 6;

for i = 1:total_exp
    
    file = file_list(i);
    % tmp_results = load(fullfile(datapath, file.name));
    tmp_results = load(file.name);
    success_counter = success_counter + tmp_results.success;
    % slow_sys_counters = slow_sys_counters + tmp_results.slow_system;
    exp_data_name = file.name(1:end-11) + "data.csv";
    tab = readtable(exp_data_name, VariableNamingRule="preserve");

    OS_tmp = max(tab.omega-stepsize)/stepsize*100;
    OS_list(i,1) = OS_tmp;
    OS_list(i,2) = tmp_results.success;

    if (show_good) && (tmp_results.success)
        figure
        ax1 = subplot(2,1,1);
        hold on
        grid on
        plot(tab.t, tab.r, "DisplayName","Omega ref")
        plot(tab.t, tab.omega, "DisplayName","Omega")
        xlabel("time [s]")
        ylabel("speed [rpm]")
        legend()

        ax2 = subplot(2,1,2);
        hold on
        grid on
        plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
        plot(tab.t, tab.iq, "DisplayName","iq")
        plot(tab.t, tab.id, "DisplayName","id")
        xlabel("time [s]")
        ylabel("current [A]")
        legend()
        linkaxes([ax1, ax2], 'x')
        input("next?")
        close all

    end
    % 
    % if OS_tmp > 20
    %     figure
    %     ax1 = subplot(2,1,1);
    %     hold on
    %     grid on
    %     plot(tab.t, tab.r, "DisplayName","Omega ref")
    %     plot(tab.t, tab.omega, "DisplayName","Omega")
    %     legend()
    % 
    %     ax2 = subplot(2,1,2);
    %     hold on
    %     grid on
    %     plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
    %     plot(tab.t, tab.iq, "DisplayName","iq")
    %     plot(tab.t, tab.id, "DisplayName","id")
    %     legend()
    % end 





    if ~tmp_results.success
        if show_bad
            figure
            ax1 = subplot(2,1,1);
            hold on
            grid on
            plot(tab.t, tab.r, "DisplayName","Omega ref")
            plot(tab.t, tab.omega, "DisplayName","Omega")
            xlabel("time [s]")
            ylabel("speed [rpm]")
            legend()

            ax2 = subplot(2,1,2);
            hold on
            grid on
            plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
            plot(tab.t, tab.iq, "DisplayName","iq")
            plot(tab.t, tab.id, "DisplayName","id")
            xlabel("time [s]")
            ylabel("current [A]")
            legend()
            linkaxes([ax1, ax2], 'x')
        end
            

        if max(tab.omega) < stepsize-eps
            limited_speed_counter = limited_speed_counter + 1;

            if plot_example == 0
                figure
                ax1 = subplot(2,1,1);
                hold on
                grid on
                plot(tab.t, tab.r, "DisplayName","Omega ref")
                plot(tab.t, tab.omega, "DisplayName","Omega")
                xlabel("time [s]")
                ylabel("speed [rpm]")
                legend()
    
                ax2 = subplot(2,1,2);
                hold on
                grid on
                plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
                plot(tab.t, tab.iq, "DisplayName","iq")
                plot(tab.t, tab.id, "DisplayName","id")
                xlabel("time [s]")
                ylabel("current [A]")
                legend()
                linkaxes([ax1, ax2], 'x')
                plot_example = 1;
            end

        elseif all(tab.iq_ref(tab.t>0.1 & tab.t<T_s_th)> 4)
            slow_sys_counters = slow_sys_counters + 1;

            if plot_example == 1
                figure
                ax1 = subplot(2,1,1);
                hold on
                grid on
                plot(tab.t, tab.r, "DisplayName","Omega ref")
                plot(tab.t, tab.omega, "DisplayName","Omega")
                xlabel("time [s]")
                ylabel("speed [rpm]")
                legend()
    
                ax2 = subplot(2,1,2);
                hold on
                grid on
                plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
                plot(tab.t, tab.iq, "DisplayName","iq")
                plot(tab.t, tab.id, "DisplayName","id")
                xlabel("time [s]")
                ylabel("current [A]")
                legend()
                linkaxes([ax1, ax2], 'x')
                plot_example = 2;
            end

        elseif max(tab.omega(tab.t<=0.1)) >= stepsize - eps
            fast_sys_counter = fast_sys_counter + 1;
            if plot_example == 2
                figure
                ax1 = subplot(2,1,1);
                hold on
                grid on
                plot(tab.t, tab.r, "DisplayName","Omega ref")
                plot(tab.t, tab.omega, "DisplayName","Omega")
                xlabel("time [s]")
                ylabel("speed [rpm]")
                legend()
    
                ax2 = subplot(2,1,2);
                hold on
                grid on
                plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
                plot(tab.t, tab.iq, "DisplayName","iq")
                plot(tab.t, tab.id, "DisplayName","id")
                xlabel("time [s]")
                ylabel("current [A]")
                legend()
                linkaxes([ax1, ax2], 'x')
                plot_example = 3;
            end

        else
            just_bad_counter = just_bad_counter + 1;

            if plot_example == 3
                figure
                ax1 = subplot(2,1,1);
                hold on
                grid on
                plot(tab.t, tab.r, "DisplayName","Omega ref")
                plot(tab.t, tab.omega, "DisplayName","Omega")
                xlabel("time [s]")
                ylabel("speed [rpm]")
                legend()
    
                ax2 = subplot(2,1,2);
                hold on
                grid on
                plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
                plot(tab.t, tab.iq, "DisplayName","iq")
                plot(tab.t, tab.id, "DisplayName","id")
                xlabel("time [s]")
                ylabel("current [A]")
                legend()
                linkaxes([ax1, ax2], 'x')
                plot_example = 4;
            end


            % figure
            % ax1 = subplot(2,1,1);
            % hold on
            % grid on
            % plot(tab.t, tab.r, "DisplayName","Omega ref")
            % plot(tab.t, tab.omega, "DisplayName","Omega")
            % legend()
            % 
            % ax2 = subplot(2,1,2);
            % hold on
            % grid on
            % plot(tab.t, tab.iq_ref, "DisplayName","iq ref")
            % plot(tab.t, tab.iq, "DisplayName","iq")
            % plot(tab.t, tab.id, "DisplayName","id")
            % legend()
            % linkaxes([ax1, ax2], 'x')
        end


        


    end


end



fprintf("of all %d experiments:\n", total_exp)
fprintf(" > %d were successful\n", success_counter)
fprintf(" > %d were system with limited maximum speed\n", limited_speed_counter)
fprintf(" > %d were system with a dynamic too slow\n", slow_sys_counters)
fprintf("\n")
fprintf(" > %d were system with a fast dynamic\n", fast_sys_counter)
fprintf(" > %d were otherwise failed attempts\n", just_bad_counter)


disp(success_counter + limited_speed_counter + slow_sys_counters + fast_sys_counter + just_bad_counter)




