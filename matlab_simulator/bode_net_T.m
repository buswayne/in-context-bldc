clc 
clear
close all

datapath = "bode_analysis_new_dataset_short_noise_h10_30k_H10H_v2";
save_path = "bode_figures_T";
save_figures = false;

[~, ~] = mkdir(save_path);
delay = 7;
stepsize_list = [500,1000,1500,2000];
inertia_mult_list = [0.5, 1, 2, 5];

folder_list = dir(sprintf('%s',datapath));
x_limits = [0.4*2*pi, 100*2*pi];

del_log_list = [];

for i = 1:length(folder_list)
    if strcmp(folder_list(i).name,'desktop.ini') | strcmp(folder_list(i).name,'.') | strcmp(folder_list(i).name,'..') | strcmp(folder_list(i).name,'.DS_Store' ) | ~(contains(folder_list(i).name, "csv"))      % 'desktop.ini' is a hidden file created by google drive in each directory. 
        del_log_list(end+1) = i;                                                                                    % It is deleted from the list since it is not useful for the program.
    end
end

folder_list(del_log_list) = [];

metadata = zeros(2,length(folder_list)); % 1:setpoint, 2:inertia multiplier

bodes = struct;


for i = 1:length(folder_list)
    
    tab = readtable(fullfile(datapath, folder_list(i).name));

    filename = folder_list(i).name;
    filename = filename(1:end-4);
    tmp = split(filename,"_");
    setpoint = str2double(tmp{2});
    inertia_mult_str = tmp{4};
    if inertia_mult_str(1) == '0'
        inertia_mult = str2double(inertia_mult_str(2:end))/10;
    else
        inertia_mult = str2double(tmp{4});
    end

    time = tab.t;
    omega = tab.omega(time>=delay);
    ref = tab.r(time>=delay);
    error = ref - omega;
    iq_ref = tab.iq_ref(time>=delay);
    time = time(time>=delay);
    time = time - time(1);
    
    figure
    subplot(311)
    hold on
    plot(time,ref)
    plot(time,omega)
    ylabel("Speed [rpm]")
    subplot(312)
    hold on
    plot(time,error)
    ylabel("Error [rpm]")
    subplot(313)
    plot(time,iq_ref)
    ylabel("iq_ref [A]")
    xlabel("Time [s]")
    
    if save_figures
        saveas(gcf, fullfile(save_path, filename+".png"))
    end
    close all
    
    
    % figure
    Ts = time(2)-time(1);
    data = iddata(omega, ref, Ts);
    g = spa(data);        
    g_etfe = etfe(data);        
    % bode(g);
    % xlim(x_limits)
    [mag, phase, wout] = bode(g);
    [mag_e, phase_e, wout_e] = bode(g_etfe);

    % N = 2^nextpow2(length(error)/8);
    % [cxy, f] = mscohere(error, iq_ref, hann(N), N/2, N, 1/Ts);
    % figure; semilogx(f, cxy); ylim([0 1]); grid on;  % sanity check



    bodes.(filename).mag = mag;
    bodes.(filename).phase = phase;
    bodes.(filename).wout = wout;

    bodes.(filename).mag_e = mag_e;
    bodes.(filename).phase_e = phase_e;
    bodes.(filename).wout_e = wout_e;



end



fields = string(fieldnames(bodes));




for stepsize = stepsize_list

    figure
    subplot(211)
    hold on
    subplot(212)
    hold on

    for j = 1:length(fields)
        field = fields(j);

        tmp = split(field,"_");
        setpoint = str2double(tmp{2});
        inertia_mult_str = tmp{4};
        if inertia_mult_str(1) == '0'
            inertia_mult = str2double(inertia_mult_str(2:end))/10;
        else
            inertia_mult = str2double(tmp{4});
        end
        if setpoint ~= stepsize
            continue
        end
        disp_name = sprintf("inertia x %.1f", inertia_mult);

        subplot(211)
        hold on 
        plot(bodes.(field).wout(:), bodes.(field).mag(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes.(field).wout(:), bodes.(field).phase(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    legend()
    suptit = sprintf("behavior around %d rpm (filtered) (T)", stepsize);
    title(suptit)

    subplot(212)
    xlim(x_limits)

    fig_tit = sprintf("stepsize_%d_T_filt.png", stepsize);
    if save_figures
        saveas(gcf, fullfile(save_path, fig_tit))
    end


end

for inertia_mult_curr = inertia_mult_list

    figure
    subplot(211)
    hold on
    subplot(212)
    hold on

    for j = 1:length(fields)
        field = fields(j);

        tmp = split(field,"_");
        setpoint = str2double(tmp{2});
        inertia_mult_str = tmp{4};
        if inertia_mult_str(1) == '0'
            inertia_mult = str2double(inertia_mult_str(2:end))/10;
        else
            inertia_mult = str2double(tmp{4});
        end
        if inertia_mult ~= inertia_mult_curr
            continue
        end
        disp_name = sprintf("at %d rpm", setpoint);

        subplot(211)
        hold on 
        plot(bodes.(field).wout(:), bodes.(field).mag(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes.(field).wout(:), bodes.(field).phase(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    legend('Location','southeast')
    suptit = sprintf("behavior for inertia x %.1f (filtered) (T)", inertia_mult_curr);
    title(suptit)

    subplot(212)
    xlim(x_limits)

    fig_tit = sprintf("inertia_%.1f_T_filt", inertia_mult_curr);
    fig_tit = replace(fig_tit, ".", "_") + ".png";
    if save_figures
        saveas(gcf, fullfile(save_path, fig_tit))
    end

end





for stepsize = stepsize_list

    figure
    subplot(211)
    hold on
    subplot(212)
    hold on

    for j = 1:length(fields)
        field = fields(j);

        tmp = split(field,"_");
        setpoint = str2double(tmp{2});
        inertia_mult_str = tmp{4};
        if inertia_mult_str(1) == '0'
            inertia_mult = str2double(inertia_mult_str(2:end))/10;
        else
            inertia_mult = str2double(tmp{4});
        end
        if setpoint ~= stepsize
            continue
        end
        disp_name = sprintf("inertia x %.1f", inertia_mult);

        subplot(211)
        hold on 
        plot(bodes.(field).wout_e(:), bodes.(field).mag_e(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes.(field).wout_e(:), bodes.(field).phase_e(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    legend()
    suptit = sprintf("behavior around %d rpm (T)", stepsize);
    title(suptit)

    subplot(212)
    xlim(x_limits)

    fig_tit = sprintf("stepsize_%d_T_etfe.png", stepsize);
    if save_figures
        saveas(gcf, fullfile(save_path, fig_tit))
    end

end

for inertia_mult_curr = inertia_mult_list

    figure
    subplot(211)
    hold on
    subplot(212)
    hold on

    for j = 1:length(fields)
        field = fields(j);

        tmp = split(field,"_");
        setpoint = str2double(tmp{2});
        inertia_mult_str = tmp{4};
        if inertia_mult_str(1) == '0'
            inertia_mult = str2double(inertia_mult_str(2:end))/10;
        else
            inertia_mult = str2double(tmp{4});
        end
        if inertia_mult ~= inertia_mult_curr
            continue
        end
        disp_name = sprintf("at %d rpm", setpoint);

        subplot(211)
        hold on 
        plot(bodes.(field).wout_e(:), bodes.(field).mag_e(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes.(field).wout_e(:), bodes.(field).phase_e(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    legend('Location','southeast')
    suptit = sprintf("behavior for inertia x %.1f (T)", inertia_mult_curr);
    title(suptit)

    subplot(212)
    xlim(x_limits)

    fig_tit = sprintf("inertia_%.1f_T_etfe", inertia_mult_curr);
    fig_tit = replace(fig_tit, ".", "_") + ".png";
    if save_figures
        saveas(gcf, fullfile(save_path, fig_tit))
    end

end

