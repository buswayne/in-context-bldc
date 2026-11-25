clc
clear
close all

bodes_T = load("bode_figures_T\bodes_T.mat");
bodes_P = load("bode_figures_P\bodes_P.mat");

bodes_T = bodes_T.bodes;
bodes_P = bodes_P.bodes;

exp_list = fieldnames(bodes_P);
save_path = "bode_figures_ICC";
save_figures = true;

[~, ~] = mkdir(save_path);

wmax = 50 * 2 * pi;
wmin = 0.1 * 2 * pi;
x_limits = [wmin,wmax];

% g_raw_ICC_list = [];
% g_filt_ICC_list = [];
bodes_ICC = struct;

for i = 1:length(exp_list)

    exp_curr = exp_list{i};
    tmp = split(exp_curr,"_");
    setpoint = str2double(tmp{2});
    inertia_mult_str = tmp{4};
    if inertia_mult_str(1) == '0'
        inertia_mult = str2double(inertia_mult_str(2:end))/10;
    else
        inertia_mult = str2double(tmp{4});
    end


    g_raw_P = bodes_P.(exp_curr).g_raw;
    g_raw_T = bodes_T.(exp_curr).g_raw;

    g_filt_P = bodes_P.(exp_curr).g_filt;
    g_filt_T = bodes_T.(exp_curr).g_filt;

    g_raw_ICC = g_raw_T / (g_raw_P * (1 - g_raw_T));
    g_filt_ICC = g_filt_T / (g_filt_P * (1 - g_filt_T));

    % g_raw_ICC_list = [g_raw_ICC_list, g_raw_ICC];
    % g_filt_ICC_list = [g_filt_ICC_list, g_filt_ICC];
    bodes_ICC.(exp_curr).g_raw = g_raw_ICC;
    bodes_ICC.(exp_curr).g_filt = g_filt_ICC;

    [mag, phase, wout] = bode(g_filt_ICC);
    [mag_e, phase_e, wout_e] = bode(g_raw_ICC);

    bodes_ICC.(exp_curr).mag = mag;
    bodes_ICC.(exp_curr).phase = phase;
    bodes_ICC.(exp_curr).wout = wout;

    bodes_ICC.(exp_curr).mag_e = mag_e;
    bodes_ICC.(exp_curr).phase_e = phase_e;
    bodes_ICC.(exp_curr).wout_e = wout_e;


    % figure
    % bode(g_raw_T, g_raw_P, g_raw_ICC)
    % legend()
    % title([exp_curr, ' raw'])
    % drawnow
    % 
    % figure
    % bode(g_filt_T, g_filt_P, g_filt_ICC)
    % legend()
    % title([exp_curr, ' filt'])


end

%ICC
figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_ICC.(exp_list{i}).g_raw;
    g_curr.InputName = [];
    g_curr.OutputName = [];

    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_ICC_raw.png");
suptit = sprintf("ICC behavior (raw)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end


figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_ICC.(exp_list{i}).g_filt;
    g_curr.InputName = [];
    g_curr.OutputName = [];
    
    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_ICC_filt.png");
suptit = sprintf("ICC behavior (filtered)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end



% plant
figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_P.(exp_list{i}).g_raw;
    g_curr.InputName = [];
    g_curr.OutputName = [];

    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_P_raw.png");
suptit = sprintf("Plant behavior (raw)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end


figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_P.(exp_list{i}).g_filt;
    g_curr.InputName = [];
    g_curr.OutputName = [];
    
    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_P_filt.png");
suptit = sprintf("Plant behavior (filtered)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end


% T
figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_T.(exp_list{i}).g_raw;
    g_curr.InputName = [];
    g_curr.OutputName = [];

    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_T_raw.png");
suptit = sprintf("Closed-loop behavior (raw)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end


figure
hold on
for i = 1:length(exp_list)

    g_curr = bodes_T.(exp_list{i}).g_filt;
    g_curr.InputName = [];
    g_curr.OutputName = [];
    
    bode(g_curr, 'k')
end
xlim([wmin,wmax])
fig_tit = sprintf("Full_T_filt.png");
suptit = sprintf("Closed-loop behavior (filtered)");
title(suptit)
if save_figures
    saveas(gcf, fullfile(save_path, fig_tit))
end




















stepsize_list = [500,1000,1500,2000];
inertia_mult_list = [0.5, 1, 2, 5];




for stepsize = stepsize_list

    figure
    subplot(211)
    hold on
    subplot(212)
    hold on

    for j = 1:length(exp_list)
        field = exp_list{j};

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
        plot(bodes_ICC.(field).wout(:), bodes_ICC.(field).mag(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes_ICC.(field).wout(:), bodes_ICC.(field).phase(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    xscale log
    yscale log
    legend('Location','southeast')
    suptit = sprintf("behavior around %d rpm (filtered) (ICC)", stepsize);
    title(suptit)

    subplot(212)
    xscale log
    xlim(x_limits)

    fig_tit = sprintf("stepsize_%d_ICC_filt.png", stepsize);
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

    for j = 1:length(exp_list)
        field = exp_list{j};

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
        plot(bodes_ICC.(field).wout(:), bodes_ICC.(field).mag(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes_ICC.(field).wout(:), bodes_ICC.(field).phase(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xscale log
    yscale log
    xlim(x_limits)
    legend('Location','southeast')
    suptit = sprintf("behavior for inertia x %.1f (filtered) (ICC)", inertia_mult_curr);
    title(suptit)

    subplot(212)
    xscale log
    xlim(x_limits)

    fig_tit = sprintf("inertia_%.1f_ICC_filt", inertia_mult_curr);
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

    for j = 1:length(exp_list)
        field = exp_list{j};

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
        plot(bodes_ICC.(field).wout_e(:), bodes_ICC.(field).mag_e(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes_ICC.(field).wout_e(:), bodes_ICC.(field).phase_e(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xlim(x_limits)
    xscale log
    yscale log
    legend('Location','southeast')
    suptit = sprintf("behavior around %d rpm (raw) (ICC)", stepsize);
    title(suptit)

    subplot(212)
    xscale log
    xlim(x_limits)

    fig_tit = sprintf("stepsize_%d_ICC_raw.png", stepsize);
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

    for j = 1:length(exp_list)
        field = exp_list{j};

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
        plot(bodes_ICC.(field).wout_e(:), bodes_ICC.(field).mag_e(:), 'DisplayName',disp_name)
        ylabel("Mag")
        subplot(212)
        hold on
        plot(bodes_ICC.(field).wout_e(:), bodes_ICC.(field).phase_e(:))
        ylabel("Phase")
        xlabel("Frequency [rad/s]")
    
    end

    subplot(211)
    xscale log
    yscale log
    xlim(x_limits)
    legend('Location','southeast')
    suptit = sprintf("behavior for inertia x %.1f (raw) (ICC)", inertia_mult_curr);
    title(suptit)

    subplot(212)
    xscale log
    xlim(x_limits)

    fig_tit = sprintf("inertia_%.1f_ICC_raw", inertia_mult_curr);
    fig_tit = replace(fig_tit, ".", "_") + ".png";
    if save_figures
        saveas(gcf, fullfile(save_path, fig_tit))
    end

end