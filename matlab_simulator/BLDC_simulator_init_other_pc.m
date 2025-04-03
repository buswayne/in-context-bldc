clear
clc
close all
tic
pause on
temp_name = strsplit(pwd,'in-context-bldc');
% savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");

perturbation_percent = 10;

perturbation = perturbation_percent / 100;
savepath_tmp = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
folder_name = sprintf('%02.0f_percent_shorter_steps', perturbation_percent);
savepath = fullfile(savepath_tmp, folder_name);
[tmp, tmp2] = mkdir(savepath);

% savepath = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated\15_percent";
speed_loop = 1;
current_loop = 1;
T = 10;
Ts = 1e-4;
time = 0:Ts:T-Ts;
Min_value = 0;
Max_value = 3000; %in rpm
Min_duration = 1;
Max_duration = 3;


N_exp = 1000;

mdl = 'BLDC_simulator2';
% Simulink.BlockDiagram.buildRapidAcceleratorTarget(mdl);

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)

    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
    set_parameters_corrected_perturbed
    % set_parameters
    
    reference_speed = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
    reference_speed = reference_speed / 30 * pi; %in rad/s
    
    speed_input.time = time;
    speed_input.signals.values = reference_speed;
    load_input.time = time;
    load_input.signals.values = zeros(length(time),1);
    current_input.time = time;
    current_input.signals.values = zeros(length(time),1);
    voltage_d_input.time = time;
    voltage_d_input.signals.values = zeros(length(time),1);
    voltage_q_input.time = time;
    voltage_q_input.signals.values = zeros(length(time),1);

    
    % in = Simulink.SimulationInput(mdl);
    % in = setModelParameter(in,SimulationMode="rapid-accelerator");
    % in = setModelParameter(in,RapidAcceleratorUpToDateCheck="off");
    % output = parsim(in);
    output = sim(mdl);
    
    
    output_clean.t = output.output.time;
    output_clean.theta = output.output.signals.values(:,1);
    output_clean.omega = output.output.signals.values(:,2);
    output_clean.r = output.output.signals.values(:,3);
    output_clean.i_d = output.output.signals.values(:,4);
    output_clean.i_q = output.output.signals.values(:,5);
    output_clean.i_q_ref = output.output.signals.values(:,6);
    output_clean.v_d = output.output.signals.values(:,7);
    output_clean.v_q = output.output.signals.values(:,8);
    
    out_tab = struct2table(output_clean);
    
    exp_name = "Experiment__" + now_string + ".csv";
    writetable(out_tab,fullfile(savepath,exp_name));
    
    % if rem(idx_exp, 100) == 0
    %     pause(1200)
    % end
    
    % figure
    % ax1 = subplot(3,1,1);
    % hold on
    % grid on
    % plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
    % plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
    % legend(["Omega ref", "Omega"])
    % 
    % 
    % ax2 = subplot(3,1,2);
    % hold on
    % grid on
    % plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
    % plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
    % plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
    % legend()
    % 
    % ax3 = subplot(3,1,3);
    % hold on
    % grid on
    % plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
    % plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
    % legend()
    % linkaxes([ax1, ax2, ax3], 'x')
end

toc